"""Pure, hardware-free GNSS compatibility state and experimental pose model.

Existing UDP fields always remain GNSS observations. Shadow estimates are never
substituted into legacy lat/lon/heading or used to authorize movement.
"""
from __future__ import annotations

import math
import statistics

from tractor_rpi import position_gnss_protocol as protocol
from tractor_rpi.imu_sensor import (
    FIELDS as IMU_FIELDS, YawTracker, bias_statistics, finite,
)

PORTS = (6002, 6009, 6010, 6013)
LEFT_SERIAL = "5879620"
RIGHT_SERIAL = "F5CB3C7E51514A4E43202020FF142006"
PROVISIONAL_METRES_PER_SENSOR_REV = 1.59593
POSITION_IMU_FIELDS=IMU_FIELDS+("gnss_input_transport","gnss_position_age_s")


def parse_base_nmea(line):
    """Return a state patch only for a checksum-valid, supported sentence."""
    sentence = protocol.parse_checksum_valid_nmea_type(line)
    if sentence is None:
        return None
    try:
        fields = line.decode("ascii").strip().split("*", 1)[0].split(",")
        if sentence.endswith("GGA"):
            fix = int(fields[6])
            optional = lambda i, fn: fn(fields[i]) if len(fields)>i and fields[i] else None
            patch = {"fix_quality": protocol.FIX_QUALITY.get(fix, "Unknown"),
                     "base_numSV_used_gga": optional(7,int), "hdop": optional(8,float),
                     "diff_age": optional(13,float)}
            if fix in (1,2,4,5) and fields[2] and fields[4]:
                def coordinate(raw, direction, allowed):
                    if direction not in allowed:
                        raise ValueError("invalid coordinate direction")
                    value=float(raw); degrees=int(value/100); minutes=value-degrees*100
                    if not 0<=minutes<60:raise ValueError("invalid coordinate minutes")
                    result=degrees+minutes/60
                    return -result if direction in ("S","W") else result
                lat=coordinate(fields[2],fields[3],("N","S"))
                lon=coordinate(fields[4],fields[5],("E","W"))
                if not finite(lat) or not finite(lon) or not -90<=lat<=90 or not -180<=lon<=180:
                    raise ValueError("invalid coordinates")
                patch.update(lat=lat,lon=lon)
            if any(isinstance(v,float) and not finite(v) for v in patch.values()):return None
            return "gga",patch
        if sentence.endswith("RMC") and fields[2]=="A":
            speed=float(fields[7])*0.514444
            source="RMC"
        elif sentence.endswith("VTG"):
            speed=float(fields[7])/3.6
            source="VTG"
        else:return None
        if not finite(speed) or speed<0:return None
        return "speed",{"speed_mps":round(speed,3),"speed_source":source}
    except (IndexError,ValueError,UnicodeDecodeError):
        return None


def parse_encoder(line, expected_side):
    text=line.decode("ascii").strip()
    if not text or text.startswith("#"):return "banner",{"raw":text}
    pieces=text.split(",")
    if len(pieces)<3 or pieces[1]!=expected_side:raise ValueError("encoder side/schema mismatch")
    ms=int(pieces[2])
    if not 0<=ms<=0xFFFFFFFF:raise ValueError("invalid encoder device clock")
    if pieces[0] in ("ERR","WARN"):
        return pieces[0].lower(),{"device_ms":ms,"raw":text}
    if pieces[0]!="ENC" or len(pieces)!=8:raise ValueError("invalid ENC field count")
    pos,ticks=int(pieces[3]),int(pieces[5])
    values=[float(pieces[i]) for i in (4,6,7)]
    if not 0<=pos<16384 or not -8192<=ticks<=8192 or not all(finite(v) for v in values):
        raise ValueError("invalid encoder counts/numbers")
    return "sample",dict(device_ms=ms,position=pos,total_rot=values[0],delta_ticks=ticks,
                         reported_speed_mps=values[1],reported_delta_m=values[2],raw=text)


class PoseModel:
    def __init__(self, log, scales=None, calibration_id=None):
        self.log=log
        self.gps=protocol.initial_state()
        self.gps_times={}
        self.imu_start=None
        self.last_imu=None
        self.bias=None
        self.calibration_allowed=False
        self.calibration=None
        self.cal_gyro=[]; self.cal_accel=[]
        self.yaw=YawTracker()
        self.imu_count=0
        self.heading=None
        self.heading_anchor=None
        self.heading_anchor_count=None
        self.origin=None
        self.position=None
        self.position_method="HELD_UNSUPPORTED"
        self.position_anchor=None
        self.travel_since_anchor=0.
        self.scales=scales or {"L":None,"R":None}
        self.calibration_id=calibration_id
        self.wheels={side:dict(last=None,device_ms=None,valid=False,continuous=False,
                              pending=0.,provisional_total_m=0.,samples=0,discontinuities=0,
                              last_ticks=None,suppress_next=False,pending_samples=0,error_mono=-math.inf) for side in ("L","R")}

    def gps_quality(self, now):
        age=lambda key: now-self.gps_times[key] if key in self.gps_times else None
        position_age,heading_age=age("position"),age("heading")
        g=self.gps
        position=(position_age is not None and 0<=position_age<=1 and g["fix_quality"]=="RTK Fixed"
                  and finite(g["lat"]) and finite(g["lon"]))
        heading=(heading_age is not None and 0<=heading_age<=1 and g["headValid"] is True
                 and g["carrier"]=="fixed" and g["relpos_valid"] is True and g["relpos_gnss_fix_ok"] is True
                 and g["relpos_ref_pos_miss"] is False and g["relpos_ref_obs_miss"] is False
                 and finite(g["relpos_heading_accuracy_deg"]) and 0<=g["relpos_heading_accuracy_deg"]<=1
                 and finite(g["relpos_length_m"]) and .8<=g["relpos_length_m"]<=1.3
                 and finite(g["heading_deg"]) and 0<=g["heading_deg"]<360)
        return dict(position_healthy=bool(position),heading_healthy=bool(heading),
                    position_age_s=position_age,heading_age_s=heading_age)

    def apply_gps(self, event):
        kind,data=event["kind"],event["data"]
        mono,utc=event["mono"],event["utc"]
        if kind in ("gga","speed"):
            self.gps.update(data)
            if kind=="gga":
                self.gps["timestamp"]=utc
                if "lat" in data:self.gps_times["position"]=mono
            else:
                self.gps["speed_timestamp"]=utc; self.gps["speed_update_count"]+=1
        elif kind=="heading":
            self.gps.update(headValid=data["headValid"],carrier=data["carrier"],
                relposned_timestamp=utc,relposned_itow_ms=data["iTOW_ms"],relpos_length_m=data["length_m"],
                relpos_heading_accuracy_deg=data["accHead_deg"],relpos_gnss_fix_ok=data["gnssFixOK"],
                relpos_diff_solution=data["diffSoln"],relpos_valid=data["relPosValid"],
                relpos_moving=data["isMoving"],relpos_ref_pos_miss=data["refPosMiss"],
                relpos_ref_obs_miss=data["refObsMiss"],relpos_normalized=data["relPosNormalized"],timestamp=utc)
            self.gps["relposned_count"]+=1
            value=protocol.expected_heading_error_deg(data)
            self.gps["expectedErrDeg"]=value if finite(value) else None
            if data["headValid"]:self.gps["heading_deg"]=data["heading_deg"]
            self.gps_times["heading"]=mono
        elif kind=="satellites":
            prefix=event["source"]+"_"
            self.gps.update({prefix+key:value for key,value in data.items()})
            self.gps[prefix+"satellite_timestamp"]=utc
        elif kind=="heading_nmea":
            self.gps["heading_nmea_count"]+=1
            self.gps["heading_nmea_last_type"]=data
            self.gps["heading_nmea_timestamp"]=utc
            counts=self.gps["heading_nmea_types"]
            counts[data]=counts.get(data,0)+1
        self.correct_from_gnss(mono)

    def correct_from_gnss(self, now):
        quality=self.gps_quality(now)
        g=self.gps
        if quality["position_healthy"] and self.position_anchor!=self.gps_times["position"]:
            if self.origin is None:self.origin=(g["lat"],g["lon"])
            self.position=(g["lat"],g["lon"])
            self.position_method="GNSS"
            self.position_anchor=self.gps_times["position"]
            self.travel_since_anchor=0.
            for wheel in self.wheels.values():wheel["pending"]=0.;wheel["pending_samples"]=0
        # Experimental five-second heading anchors affect shadow output only.
        if (self.bias is not None and self.last_imu is not None
            and quality["position_healthy"] and quality["heading_healthy"]
            and (self.heading_anchor is None or now-self.heading_anchor>=5.)
            and self.heading_anchor_count!=g["relposned_count"]):
            self.heading=g["heading_deg"]
            self.heading_anchor=now
            self.heading_anchor_count=g["relposned_count"]
            self.log("events",dict(event="shadow_heading_anchor",mono=now,
                heading_deg=self.heading,gnss_heading_timestamp=g["relposned_timestamp"],
                note="Decode-time alignment; experimental; not a control input"))

    def apply_imu(self, event):
        mono=event["mono"]
        raw,gyro,accel=event["data"]
        if self.imu_start is None:self.imu_start=mono
        delta=0. if self.last_imu is None else mono-self.last_imu
        if delta<0:raise ValueError("out-of-order IMU samples")
        self.last_imu=mono
        phase=("waiting_stationary" if self.bias is None and not self.calibration_allowed else
               "settling" if mono-self.imu_start<1 else "calibration" if self.bias is None else "observation")
        if phase=="calibration":
            self.cal_gyro.append(gyro);self.cal_accel.append(accel)
            if mono-self.imu_start>=11:
                means,std=bias_statistics(self.cal_gyro)
                accel_std=[statistics.stdev(a) for a in zip(*self.cal_accel)]
                magnitude=statistics.mean(math.sqrt(sum(v*v for v in a)) for a in self.cal_accel)
                if len(self.cal_gyro)<800 or max(std)>.5 or max(abs(v) for v in means)>5 or max(accel_std)>.03 or not .85<=magnitude<=1.15:
                    raise RuntimeError("Stationary IMU calibration failed; keep engine off and retry")
                self.bias=means
                self.calibration=dict(samples=len(self.cal_gyro),bias_dps=means,stddev_dps=std,
                                      accel_stddev_g=accel_std,accel_mean_magnitude_g=magnitude)
                self.log("events",dict(event="imu_calibration_passed",mono=mono,**self.calibration))
        row=dict(utc=event["utc"],monotonic_s=mono,sample=self.imu_count,phase=phase,dt_s=delta,
                 gnss_socket_available=None,gnss_packet_age_s=None,gnss_input_transport="direct_serial",
                 gnss_position_age_s=self.gps_quality(mono)["position_age_s"],
                 gnss_heading_age_s=self.gps_quality(mono)["heading_age_s"],
                 gnss_heading_usable=int(all(self.gps_quality(mono)[k] for k in ("position_healthy","heading_healthy"))))
        self.imu_count+=1
        row.update(zip(IMU_FIELDS[6:12],raw));row.update(zip(IMU_FIELDS[12:15],gyro));row.update(zip(IMU_FIELDS[15:18],accel))
        for key in IMU_FIELDS:
            if key.startswith("gnss_") and key[5:] in self.gps:row[key]=self.gps[key[5:]]
        if phase=="observation":
            corrected=[v-b for v,b in zip(gyro,self.bias)]
            row.update(zip(IMU_FIELDS[18:21],corrected))
            previous=self.yaw.yaw
            healthy=self.gps_quality(mono)
            gap,compass=self.yaw.update(corrected[2],delta,self.gps["heading_deg"] if healthy["position_healthy"] and healthy["heading_healthy"] else None)
            if gap:
                self.heading=None;self.heading_anchor=None
                self.log("events",dict(event="imu_gap",mono=mono,dt_s=delta,segment=self.yaw.segment))
            elif self.heading is not None:self.heading=(self.heading-(self.yaw.yaw-previous))%360
            self.correct_from_gnss(mono)
            error=None if compass is None or not row["gnss_heading_usable"] else (compass-self.gps["heading_deg"]+180)%360-180
            row.update(timing_gap=int(gap),relative_yaw_ccw_deg=self.yaw.yaw,yaw_segment=self.yaw.segment,
                       imu_compass_once_anchored_deg=compass,imu_minus_gnss_deg=error)
        self.log("imu",row)

    def apply_encoder(self, event):
        side=event["source"];wheel=self.wheels[side]
        data,kind,mono=event["data"],event["kind"],event["mono"]
        if kind!="sample":
            if kind in ("err","warn","parse_error"):
                wheel.update(valid=False,continuous=False,pending=0.,pending_samples=0,suppress_next=True,error_mono=mono)
                wheel["discontinuities"]+=1
            self.log("events",dict(event="encoder_"+kind,side=side,mono=mono,data=data))
            return
        ms=data["device_ms"]
        device_delta=None if wheel["device_ms"] is None else (ms-wheel["device_ms"])&0xFFFFFFFF
        contiguous=(wheel["last"] is not None and mono-wheel["last"]<=.5 and
                    device_delta is not None and 0<device_delta<=500 and not wheel["suppress_next"])
        delta_m=data["delta_ticks"]/16384*(self.scales[side] or PROVISIONAL_METRES_PER_SENSOR_REV)
        # WARN substitutes reported speed but leaves raw distance unchanged.
        # Never turn a flagged/discontinuous increment into trusted odometry.
        if contiguous:
            wheel["provisional_total_m"]+=delta_m
            wheel["pending"]+=delta_m
            wheel["pending_samples"]+=1
        elif wheel["last"] is not None:
            wheel["discontinuities"]+=1
            wheel.update(pending=0.,pending_samples=0,continuous=False,error_mono=mono)
        wheel.update(last=mono,device_ms=ms,valid=contiguous,suppress_next=False,last_ticks=data["delta_ticks"])
        wheel["samples"]+=1
        quality=self.gps_quality(mono)
        if contiguous and quality["position_healthy"] and self.position_anchor is not None and self.position_anchor>=wheel["error_mono"]:
            wheel["continuous"]=True
        self.log("encoders",dict(utc=event["utc"],monotonic_s=mono,side=side,
            contiguous=int(contiguous),scale_calibrated=int(self.scales[side] is not None),
            computed_delta_m=delta_m,**data))
        if all(w["valid"] and w["continuous"] and w["pending_samples"]>0 and mono-w["last"]<=.5 for w in self.wheels.values()):
            if all(self.scales.values()) and self.heading is not None and self.position is not None:
                distance=sum(w["pending"] for w in self.wheels.values())/2
                lat,lon=self.position
                angle=math.radians(self.heading)
                self.position=(lat+distance*math.cos(angle)/110540,
                               lon+distance*math.sin(angle)/(111320*math.cos(math.radians(lat))))
                self.position_method="CALIBRATED_WHEEL_IMU_EXPERIMENT"
                self.travel_since_anchor+=abs(distance)
            for w in self.wheels.values():w["pending"]=0.;w["pending_samples"]=0

    def quality(self, now):
        return dict(**self.gps_quality(now),imu_fresh=self.last_imu is not None and 0<=now-self.last_imu<=.1,
                    imu_calibrated=self.bias is not None,
                    encoders={s:dict(fresh=w["last"] is not None and 0<=now-w["last"]<=.5,
                                     valid=w["valid"],continuous=w["continuous"],samples=w["samples"],
                                     scale_calibrated=self.scales[s] is not None,discontinuities=w["discontinuities"])
                              for s,w in self.wheels.items()})

    def snapshot(self, now):
        q=self.quality(now)
        supported=q["position_healthy"] or (all(v["fresh"] and v["valid"] and v["continuous"] and v["scale_calibrated"] for v in q["encoders"].values()) and q["imu_fresh"] and self.heading is not None)
        imu_heading=q["imu_fresh"] and q["imu_calibrated"] and self.heading is not None
        heading=self.heading if imu_heading else self.gps["heading_deg"] if q["heading_healthy"] else self.heading
        return dict(schema=1,status="SHADOW_ONLY_DRAFT_UNTESTED",valid_for_control=False,
            position=None if self.position is None else dict(lat=self.position[0],lon=self.position[1],frame="WGS84"),
            heading_deg=heading,heading_supported=bool(imu_heading or q["heading_healthy"]),
            heading_mode="GNSS_ANCHORED_IMU_EXPERIMENT" if imu_heading else "GNSS" if q["heading_healthy"] else "HELD_UNSUPPORTED",
            position_supported=bool(supported),position_mode=self.position_method if supported else "HELD_UNSUPPORTED",
            heading_anchor_age_s=None if self.heading_anchor is None else now-self.heading_anchor,
            position_anchor_age_s=None if self.position_anchor is None else now-self.position_anchor,
            calibrated_travel_since_position_anchor_m=self.travel_since_anchor,
            uncertainty=dict(quantified=False,position_m=None,heading_deg=None),
            quality=q,encoder_calibration_id=self.calibration_id,
            note="Not fused into legacy ports; decode-time heading alignment remains experimental")
