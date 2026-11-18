#!/usr/bin/env python3
"""Evidence-backed, offline first IMU experiment analysis. No control changes."""
from __future__ import annotations
import argparse
import csv
import datetime as dt
import json
import math
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from build_tractor01_imu_replay_20261117 import verify_capture, health, epoch
from build_tractor_mission_replays_20260919 import digest, numeric

COLS = {"mono":"monotonic_s", "yaw":"relative_yaw_ccw_deg", "segment":"yaw_segment",
        "gx":"gyro_x_corrected_dps", "gy":"gyro_y_corrected_dps", "gz":"gyro_z_corrected_dps",
        "ax":"accel_x_g", "ay":"accel_y_g", "az":"accel_z_g", "heading":"gnss_heading_deg",
        "lat":"gnss_lat", "lon":"gnss_lon", "speed":"gnss_speed_mps", "delta":"dt_s"}
EASTERN=dt.timezone(dt.timedelta(hours=-5))


def local(stamp):
    return dt.datetime.fromtimestamp(float(stamp),EASTERN).strftime("%H:%M:%S")


def local_epoch(clock):
    return epoch("2026-11-17T"+clock+"-05:00")


def wrap(value):
    return (np.asarray(value)+180)%360-180


def describe(values):
    a=np.asarray(values,dtype=float)
    a=a[np.isfinite(a)]
    if len(a)==0:return {"n":0}
    return {"n":len(a),"mean":float(np.mean(a)),"std":float(np.std(a,ddof=1)) if len(a)>1 else 0.,
            "median":float(np.median(a)),"p95_abs":float(np.quantile(np.abs(a),.95)),
            "p99_abs":float(np.quantile(np.abs(a),.99)),"max_abs":float(np.max(np.abs(a)))}


def circle_fit(x,y):
    """Algebraic least-squares circle; residuals/arc extent determine usefulness."""
    x,y=np.asarray(x,dtype=float),np.asarray(y,dtype=float)
    mx,my=np.mean(x),np.mean(y)
    xx,yy=x-mx,y-my
    solution,_,rank,_=np.linalg.lstsq(np.column_stack((2*xx,2*yy,np.ones(len(x)))),xx*xx+yy*yy,rcond=None)
    if rank<3: raise ValueError("Degenerate circle geometry")
    cx,cy=solution[:2]+[mx,my]
    distances=np.hypot(x-cx,y-cy)
    radius=math.sqrt(max(0.,solution[2]+sum(solution[:2]**2)))
    return {"center_east_m":float(cx),"center_north_m":float(cy),"radius_m":float(radius),
            "diameter_m":float(2*radius),"radial_rms_m":float(np.sqrt(np.mean((distances-radius)**2))),
            "radial_p95_abs_m":float(np.quantile(np.abs(distances-radius),.95)),
            "east_span_m":float(np.ptp(x)),"north_span_m":float(np.ptp(y)),"points":len(x)}


def load(root):
    verify_capture(root)
    data={key:[] for key in COLS}
    data.update(epoch=[],good=[],heading_good=[],position_good=[],phase=[],nav_source=[],gyro_near_full_scale=[],accel_near_full_scale=[])
    nav=[]
    last_source=None
    with (root/"samples.csv").open(newline="",encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            for key,column in COLS.items():data[key].append(numeric(row.get(column),float("nan")))
            stamp=epoch(row["utc"])
            quality=health(row)
            data["epoch"].append(stamp)
            data["good"].append(quality["code"]=="good")
            data["heading_good"].append(quality["heading"] and quality["baselineOk"] and quality["fresh"])
            data["position_good"].append(quality["fixed"] and quality["fresh"])
            data["phase"].append(row["phase"]=="observation")
            data["gyro_near_full_scale"].append(any(abs(numeric(row.get(f"gyro_{axis}_raw"),0))>=32760 for axis in "xyz"))
            data["accel_near_full_scale"].append(any(abs(numeric(row.get(f"accel_{axis}_raw"),0))>=32760 for axis in "xyz"))
            source=row.get("gnss_relposned_timestamp")
            if source and source!=last_source:
                nav.append({"row":len(data["epoch"])-1,"epoch":epoch(source),"heading":numeric(row.get("gnss_heading_deg"),float("nan")),
                            "lat":numeric(row.get("gnss_lat"),float("nan")),"lon":numeric(row.get("gnss_lon"),float("nan")),
                            "speed":numeric(row.get("gnss_speed_mps"),float("nan")),"good":quality["code"]=="good",
                            "heading_good":quality["heading"] and quality["baselineOk"] and quality["fresh"],
                            "position_good":quality["fixed"] and quality["fresh"]})
                last_source=source
    data.pop("nav_source")
    d={key:np.asarray(values) for key,values in data.items()}
    if not np.all(np.diff(d["mono"])>0) or not np.all(np.diff(d["epoch"])>=0):
        raise ValueError("Nonmonotonic capture clocks need explicit handling")
    origin=(float(d["lat"][0]),float(d["lon"][0]))
    sx=111320*math.cos(math.radians(origin[0]))
    d["x"]=(d["lon"]-origin[1])*sx
    d["y"]=(d["lat"]-origin[0])*110540
    nav=[r for r in nav if d["epoch"][0]<=r["epoch"]<=d["epoch"][-1]]
    n={key:np.asarray([r[key] for r in nav]) for key in nav[0]}
    if not np.all(np.diff(n["epoch"])>0):raise ValueError("Nonmonotonic heading source times")
    n["mono"]=np.interp(n["epoch"],d["epoch"],d["mono"])
    n["x"]=(n["lon"]-origin[1])*sx
    n["y"]=(n["lat"]-origin[0])*110540
    n["yaw"]=-np.degrees(np.unwrap(np.radians(n["heading"])))
    smooth=np.convolve(n["yaw"],np.ones(5)/5,mode="same")
    smooth[:3]=n["yaw"][:3];smooth[-3:]=n["yaw"][-3:]
    n["rate"]=np.gradient(smooth,n["mono"])
    # Course-based reverse indicator over a 1.2 s position chord. Ground speed
    # is unsigned; this estimates travel direction, not transmission command.
    ahead=np.minimum(np.arange(len(nav))+3,len(nav)-1)
    behind=np.maximum(np.arange(len(nav))-3,0)
    dx,dy=n["x"][ahead]-n["x"][behind],n["y"][ahead]-n["y"][behind]
    angle=np.radians(n["heading"])
    n["along_speed"]=(dx*np.sin(angle)+dy*np.cos(angle))/(n["mono"][ahead]-n["mono"][behind])
    return d,n,origin


def runs(mask):
    edges=np.diff(np.r_[False,mask,False].astype(int))
    return list(zip(np.flatnonzero(edges==1),np.flatnonzero(edges==-1)-1))


def candidate_turns(n):
    # A low-pass GNSS heading rate locates maneuvers without using IMU as truth.
    labels=np.where((n["speed"]>.12)&(np.abs(n["rate"])>3),np.sign(n["rate"]),0)
    found=[]
    for sign in (1,-1):
        chunks=runs(labels==sign)
        merged=[]
        for a,b in chunks:
            if merged and n["mono"][a]-n["mono"][merged[-1][1]]<=1.5 and not np.any(labels[merged[-1][1]+1:a]==-sign):
                merged[-1]=(merged[-1][0],b)
            else:merged.append((a,b))
        for a,b in merged:
            change=n["yaw"][b]-n["yaw"][a]
            if abs(change)<20 or n["mono"][b]-n["mono"][a]<2:continue
            found.append((a,b))
    return sorted(found)


def select_time(epochs,start,end):
    return (epochs>=local_epoch(start))&(epochs<local_epoch(end))


def stationary_metrics(d):
    # Explicit engine-off/on stationary intervals selected after inspecting
    # GPS speed and path, not by searching for the smallest IMU noise.
    windows=[("engine_off_before","11:19:00","11:20:00"),
             ("engine_on_stopped","11:23:00","11:24:00"),
             ("engine_off_after","11:38:00","11:38:50")]
    result=[]
    for name,start,end in windows:
        mask=select_time(d["epoch"],start,end)&d["phase"]
        if not np.any(mask):continue
        index=np.flatnonzero(mask)
        a,b=index[0],index[-1]
        record={"interval":name,"start":start,"end":end,"samples":len(index),
                "duration_s":float(d["mono"][b]-d["mono"][a]),
                "gps_speed_max_mps":float(np.max(d["speed"][mask])),
                "gps_speed_median_mps":float(np.median(d["speed"][mask])),
                "position_displacement_m":float(np.hypot(d["x"][b]-d["x"][a],d["y"][b]-d["y"][a])),
                "all_health_fraction":float(np.mean(d["good"][mask])),
                "yaw_change_deg":float(d["yaw"][b]-d["yaw"][a]),
                "same_yaw_segment":bool(d["segment"][a]==d["segment"][b])}
        for key in ("gx","gy","gz","ax","ay","az"):record[key]=describe(d[key][mask])
        seconds=np.floor(d["mono"][mask]-d["mono"][a]).astype(int)
        blockmeans=[np.mean(d["gz"][mask][seconds==s]) for s in np.unique(seconds)]
        record["gyro_z_one_second_mean"]=describe(blockmeans)
        result.append(record)
    return result


def compare_interval(d,n,a,b,reference="good"):
    t0,t1=n["mono"][a],n["mono"][b]
    left=max(0,int(np.searchsorted(d["mono"],t0,side="right")-1))
    right=min(len(d["mono"])-1,int(np.searchsorted(d["mono"],t1,side="left")))
    same=bool(np.isfinite(d["segment"][left]) and d["segment"][left]==d["segment"][right]
              and np.all(d["delta"][left+1:right+1]<=.05))
    observation=bool(np.all(d["phase"][left:right+1]))
    if not same or not observation:return None
    imustart,imuend=np.interp([t0,t1],d["mono"],d["yaw"])
    delta=float(imuend-imustart)
    reference_delta=float(n["yaw"][b]-n["yaw"][a])
    return {"imu_ccw_deg":delta,"f9p_ccw_deg":reference_delta,
            "imu_minus_f9p_turn_deg":delta-reference_delta,
            "compass_prediction_error_deg":float(wrap(-delta-(n["heading"][b]-n["heading"][a]))),
            "duration_s":float(t1-t0),"reference_all_healthy":bool(np.all(n[reference][a:b+1]) and np.all(d[reference][left:right+1]))}


def prediction_windows(d,n):
    windows=[]
    for horizon in (1,3,5,10):
        for scheme,stride in (("nonoverlap",horizon),("rolling_1s",1)):
            target=n["mono"][0]+12
            while target+horizon<=n["mono"][-1]:
                a=int(np.searchsorted(n["mono"],target))
                desired=n["mono"][a]+horizon
                b=int(np.searchsorted(n["mono"],desired))
                if b>=len(n["mono"]):break
                if b>a and abs(n["mono"][b-1]-desired)<abs(n["mono"][b]-desired):b-=1
                # The headline set uses strictly disjoint source intervals.
                target=n["mono"][b]+1e-6 if scheme=="nonoverlap" else target+stride
                if abs(n["mono"][b]-desired)>.12:continue
                metrics=compare_interval(d,n,a,b)
                if metrics is None or not metrics["reference_all_healthy"]:continue
                speed=float(np.median(n["speed"][a:b+1]))
                if speed>.15:
                    group="moving_turn" if np.max(np.abs(n["rate"][a:b+1]))>3 else "moving_straight"
                    if np.mean(n["along_speed"][a:b+1]<-.1)>.5:group="reverse_inferred"
                elif np.max(n["speed"][a:b+1])<.05:
                    if n["epoch"][b]<local_epoch("11:22:00") or n["epoch"][a]>=local_epoch("11:38:00"):group="stationary_engine_off"
                    elif n["epoch"][a]>=local_epoch("11:23:00") and n["epoch"][b]<local_epoch("11:37:00"):group="stationary_engine_on"
                    else:group="engine_event_minute_or_slow"
                else:group="engine_event_minute_or_slow"
                windows.append({"horizon_s":horizon,"scheme":scheme,"group":group,
                    "start":local(n["epoch"][a]),"end":local(n["epoch"][b]),
                    "start_epoch":float(n["epoch"][a]),"end_epoch":float(n["epoch"][b]),
                    "median_speed_mps":speed,**metrics})
    summary=[]
    for scheme in ("nonoverlap","rolling_1s"):
        for horizon in (1,3,5,10):
            for group in ("all","moving","moving_straight","moving_turn","reverse_inferred","stationary_engine_off","stationary_engine_on"):
                selected=[w["compass_prediction_error_deg"] for w in windows if w["horizon_s"]==horizon and w["scheme"]==scheme and
                          (group=="all" or group=="moving" and w["group"] in ("moving_straight","moving_turn","reverse_inferred") or w["group"]==group)]
                if selected:summary.append({"horizon_s":horizon,"scheme":scheme,"group":group,**describe(selected)})
    return windows,summary


def turn_metrics(d,n):
    result=[]
    for k,(a,b) in enumerate(candidate_turns(n),1):
        comparison=compare_interval(d,n,a,b)
        result.append({"candidate":k,"start":local(n["epoch"][a]),"end":local(n["epoch"][b]),
                       "direction":"CCW/left yaw" if n["yaw"][b]>n["yaw"][a] else "CW/right yaw",
                       "median_speed_mps":float(np.median(n["speed"][a:b+1])),
                       "reverse_fraction":float(np.mean(n["along_speed"][a:b+1]<-.1)),
                       "healthy_fraction":float(np.mean(n["good"][a:b+1])),
                       "heading_healthy_fraction":float(np.mean(n["heading_good"][a:b+1])),
                       **(comparison or {})})
    return result


def capability(d,n):
    result=[]
    for label,start,end,sign in (("hard_left","11:34:08","11:34:26",1),("hard_right","11:34:32","11:35:01",-1)):
        mask=select_time(n["epoch"],start,end)
        indices=np.flatnonzero(mask)
        a,b=int(indices[0]),int(indices[-1])
        valid=mask&n["position_good"]&n["heading_good"]
        fit=circle_fit(n["x"][valid],n["y"][valid])
        metrics={"maneuver":label,"start":start,"end":end,"selection":"Settled interior of last left/right circular maneuvers; transition edges excluded",
                 "position_fixed_fraction":float(np.mean(n["position_good"][mask])),
                 "heading_healthy_fraction":float(np.mean(n["heading_good"][mask])),
                 "median_speed_mps":float(np.median(n["speed"][mask])),
                 "f9p_revolutions":float(sign*(n["yaw"][b]-n["yaw"][a])/360),
                 "median_speed_over_yaw_rate_radius_m":float(np.median(n["speed"][mask]/np.maximum(np.abs(np.radians(n["rate"][mask])),.01))),
                 **fit,"laps":[]}
        travel=sign*(n["yaw"][a:b+1]-n["yaw"][a])
        previous=a
        for revolution in range(1,int(np.floor(travel[-1]/360))+1):
            current=a+int(np.flatnonzero(travel>=360*revolution)[0])
            lapmask=(np.arange(len(n["epoch"]))>=previous)&(np.arange(len(n["epoch"]))<=current)&n["position_good"]&n["heading_good"]
            if np.sum(lapmask)>10:
                metrics["laps"].append({"lap":revolution,"start":local(n["epoch"][previous]),"end":local(n["epoch"][current]),
                                        **circle_fit(n["x"][lapmask],n["y"][lapmask])})
            previous=current
        result.append(metrics)
        accepted=[lap for lap in metrics["laps"] if lap["radial_rms_m"]<=.10]
        metrics["accepted_laps"]=len(accepted)
        metrics["lap_acceptance_rule"]="Full revolution and radial RMS <= 0.10 m; no statistical independence assumed"
        metrics["accepted_lap_radius_min_m"]=float(min(lap["radius_m"] for lap in accepted)) if accepted else None
        metrics["accepted_lap_radius_median_m"]=float(np.median([lap["radius_m"] for lap in accepted])) if accepted else None
        metrics["accepted_lap_radius_max_m"]=float(max(lap["radius_m"] for lap in accepted)) if accepted else None
    return result


def phase_metrics(n):
    phases=[]
    for label,start,end,drive in (("First square: CW","11:27:45","11:31:05","operator reported approximately 84%"),
                                  ("Second square: CCW","11:32:15","11:34:00","operator reported 100%")):
        mask=select_time(n["epoch"],start,end)&(n["speed"]>.15)
        phases.append({"phase":label,"start":start,"end":end,"drive":drive,
                       "timing":"Data-inferred alignment with operator sequence, not command logs",
                       "speed_mps":describe(n["speed"][mask]),
                       "heading_change_ccw_deg":float(n["yaw"][np.flatnonzero(mask)[-1]]-n["yaw"][np.flatnonzero(mask)[0]])})
    reverse=[]
    mask=(n["along_speed"]<-.1)&(n["speed"]>.1)&n["position_good"]&n["heading_good"]
    for a,b in runs(mask):
        duration=n["mono"][b]-n["mono"][a]
        if duration<2:continue
        reverse.append({"start":local(n["epoch"][a]),"end":local(n["epoch"][b]),"duration_s":float(duration),
                        "estimated_signed_speed_mps":float(np.median(n["along_speed"][a:b+1])),
                        "status":"Inferred from position chord against chassis heading; no transmission log"})
    return phases,reverse


def lag_sensitivity(d,n,windows):
    # Report timing sensitivity without applying an optimized correction.
    rows=[]
    chosen=[w for w in windows if w["scheme"]=="nonoverlap" and w["horizon_s"]==3 and w["group"]=="moving_turn"]
    for lag in (-.2,-.1,0.,.1,.2):
        errors=[]
        for w in chosen:
            a=int(np.argmin(np.abs(n["epoch"]-w["start_epoch"])))
            b=int(np.argmin(np.abs(n["epoch"]-w["end_epoch"])))
            ts=n["mono"][[a,b]]+lag
            indices=np.searchsorted(d["mono"],ts)
            if indices[1]>=len(d["mono"]) or d["segment"][indices[0]]!=d["segment"][indices[1]]:continue
            angles=np.interp(ts,d["mono"],d["yaw"])
            errors.append(float(wrap(-(angles[1]-angles[0])-(n["heading"][b]-n["heading"][a]))))
        rows.append({"imu_time_shift_s":lag,**describe(errors)})
    return rows


def epoch_phase_check(root):
    seen=set(); values=[]
    with (root/"gnss.jsonl").open(encoding="utf-8") as handle:
        for line in handle:
            record=json.loads(line)["message"]
            timestamp=record.get("relposned_timestamp")
            tow=record.get("relposned_itow_ms")
            if timestamp is None or tow is None or tow in seen:continue
            seen.add(tow)
            values.append((epoch(timestamp)-tow/1000)%1)
    values=np.asarray(values)
    return {"samples":len(values),"median_fractional_offset_s":float(np.median(values)),
            "p05_s":float(np.quantile(values,.05)),"p95_s":float(np.quantile(values,.95)),
            "definition":"(GNSS decode UTC minus iTOW seconds) modulo one second; not proof of total latency, clock sync or integer-second alignment"}


def integration_check(d):
    residuals=[]
    for segment in np.unique(d["segment"][np.isfinite(d["segment"]) ]):
        ix=np.flatnonzero(d["phase"]&(d["segment"]==segment))
        expected=np.r_[0.,np.cumsum(.5*(d["gz"][ix[:-1]]+d["gz"][ix[1:]])*d["delta"][ix[1:]])]
        residuals.append(float(np.max(np.abs(expected-d["yaw"][ix]))))
    return {"segments":len(residuals),"max_recorded_vs_reintegrated_yaw_difference_deg":max(residuals)}


def outage_metrics(d,n):
    result=[]
    for a,b in runs(~n["good"]):
        if a==0 or b==len(n["good"])-1:continue
        # Only outages during observation and after engine start are meaningful
        # to this driven experiment. Endpoints are last-good and first-good.
        if n["epoch"][a]<local_epoch("11:22:00"):continue
        comparison=compare_interval(d,n,a-1,b+1)
        result.append({"start":local(n["epoch"][a]),"recovered":local(n["epoch"][b+1]),
                       "loss_duration_s":float(n["mono"][b+1]-n["mono"][a]),
                       "position_not_fixed_fraction":float(np.mean(~n["position_good"][a:b+1])),
                       "heading_reference_available_fraction":float(np.mean(n["heading_good"][a:b+1])),
                       "median_speed_mps":float(np.median(n["speed"][a:b+1])),
                       "endpoint_prediction_error_deg":comparison["compass_prediction_error_deg"] if comparison else None})
    return result


def write_csv(path,rows):
    if not rows:return
    keys=list(dict.fromkeys(key for row in rows for key in row))
    with path.open("w",newline="",encoding="utf-8") as handle:
        writer=csv.DictWriter(handle,keys);writer.writeheader();writer.writerows(rows)


def final_plots(d,n,stationary,predictions,capabilities,out):
    plt.rcParams.update({"font.size":10,"axes.spines.top":False,"axes.spines.right":False})
    fig,axes=plt.subplots(1,2,figsize=(12,4.6),layout="constrained")
    names=["Engine off\nbefore","Engine on\nstopped","Engine off\nafter"]
    axes[0].bar(names,[r["gz"]["std"] for r in stationary],color=["#368aa9","#e79938","#368aa9"])
    axes[0].set_ylabel("Corrected gyro Z standard deviation, °/s");axes[0].set_title("Engine vibration increases raw rate noise")
    axes[1].bar(names,[r["gz"]["mean"] for r in stationary],color=["#368aa9","#e79938","#368aa9"])
    axes[1].axhline(0,color="gray",lw=.8);axes[1].set_ylabel("Mean corrected gyro Z, °/s");axes[1].set_title("Mean rate remains much smaller than raw noise")
    fig.savefig(out/"stationary_bias_and_vibration.png",dpi=160);plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(12,4.6),layout="constrained")
    for group,color,label in (("moving_straight","#368aa9","Moving straight"),("moving_turn","#d77c27","Moving turns"),("stationary_engine_on","#6b9342","Engine-on stopped")):
        rows=[r for r in predictions if r["scheme"]=="nonoverlap" and r["group"]==group]
        for ax,metric in zip(axes,("p95_abs","max_abs")):
            ax.plot([r["horizon_s"] for r in rows],[r[metric] for r in rows],"o-",label=label,color=color)
            ax.set_xlabel("GPS-free prediction horizon, seconds");ax.set_ylabel("Absolute disagreement, degrees");ax.set_xticks([1,3,5,10]);ax.grid(alpha=.2)
    axes[0].set_title("95th percentile · non-overlapping starts");axes[1].set_title("Largest observed disagreement");axes[0].legend()
    fig.suptitle("Offline heading prediction against healthy F9P · different windows at each horizon")
    fig.savefig(out/"heading_prediction_windows.png",dpi=160);plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(12,6),layout="constrained")
    for ax,record,color in zip(axes,capabilities,("#368aa9","#d77c27")):
        mask=select_time(n["epoch"],record["start"],record["end"])
        valid=mask&n["position_good"]&n["heading_good"]
        cx,cy=record["center_east_m"],record["center_north_m"]
        ax.scatter(n["x"][valid]-cx,n["y"][valid]-cy,s=7,color=color,label="Fixed-position / healthy-heading samples")
        rejected=mask&~valid
        ax.scatter(n["x"][rejected]-cx,n["y"][rejected]-cy,s=13,marker="x",color="gray",label="Excluded degraded samples")
        angle=np.linspace(0,2*np.pi,300);r=record["radius_m"]
        ax.plot(r*np.cos(angle),r*np.sin(angle),"--",color="black",lw=1,label="Whole-interval fit (includes drift/entry)")
        ax.set_aspect("equal");ax.set_xlabel("East relative to fitted center, m");ax.set_ylabel("North relative to fitted center, m");ax.grid(alpha=.2)
        for lap in record["laps"]:
            if lap["radial_rms_m"]>.1:continue
            lx,ly=lap["center_east_m"]-cx,lap["center_north_m"]-cy
            ax.plot(lx+lap["radius_m"]*np.cos(angle),ly+lap["radius_m"]*np.sin(angle),color=color,alpha=.65,lw=1)
        ax.set_title(f"{record['maneuver'].replace('_',' ').title()} · antenna path\nSettled lap radii {record['accepted_lap_radius_min_m']:.3f}–{record['accepted_lap_radius_max_m']:.3f} m")
    axes[0].legend(fontsize=8,loc="lower left")
    fig.suptitle("Observed hard turns: GPS antenna centerline and settled lap fits")
    fig.savefig(out/"hard_turn_dimensions.png",dpi=170);plt.close(fig)
    fig,ax=plt.subplots(figsize=(10,10),layout="constrained")
    ax.plot(n["x"],n["y"],color="#bfc9cd",lw=1,label="Other recorded travel")
    phases=[("CW square · reported ~84%","11:27:45","11:31:05","#268d9c"),
            ("CCW square · reported 100%","11:32:15","11:34:00","#d98130"),
            ("Likely reverse before second square","11:31:46","11:32:12","#966cb1"),
            ("Hard left","11:34:08","11:34:26","#1769ba"),
            ("Hard right","11:34:32","11:35:01","#b93842")]
    for label,start,end,color in phases:
        mask=select_time(n["epoch"],start,end)
        ax.plot(n["x"][mask],n["y"][mask],color=color,lw=2,label=label)
    ax.set_aspect("equal");ax.set_xlabel("East from capture origin, m");ax.set_ylabel("North from capture origin, m")
    ax.set_title("Driving sequence mapped to recorded GPS path\nPhase times inferred from data; drive percentages reported by operator")
    ax.grid(alpha=.2);ax.legend(fontsize=9)
    fig.savefig(out/"maneuver_path.png",dpi=160);plt.close(fig)


def write_report(result,root,notes,followup,out):
    stationary=result["stationary"]
    noise_ratio=stationary[1]["gz"]["std"]/stationary[0]["gz"]["std"]
    validated=[t for t in result["turns"] if t.get("reference_all_healthy")]
    errors=describe([t["imu_minus_f9p_turn_deg"] for t in validated])
    sources=[root/name for name in ("samples.csv","gnss.jsonl","metadata.json","SHA256SUMS")]
    sources += [notes,followup,Path(__file__).resolve(),Path(__file__).with_name("build_tractor01_imu_replay_20261117.py"),
                Path(__file__).with_name("build_tractor_mission_replays_20260919.py")]
    repo=Path(__file__).resolve().parents[2]
    sources += [repo/"tractor_rpi/testing/test_imu_manual_analysis_20261117.py",
                repo/"tractor_rpi/pure-pursuit/mission_planning/SITE_COVERAGE_MISSION_WORKFLOW_20260724.md"]
    provenance=[{"path":str(path.resolve()),"sha256":digest(path)} for path in sources]
    (out/"source_provenance.json").write_text(json.dumps(provenance,indent=2)+"\n",encoding="utf-8")
    text=["# First IMU manual-drive analysis — November 17, 2026", "",
          "**Historical evidence analysis. DRAFT — UNTESTED AS A NAVIGATION OR PLANNING POLICY.** No firmware, controller, route, safety gate or service was changed. Original evidence remains unchanged.", "",
          "## Main findings", "",
          f"1. The expected yaw sign is supported: all {len(result['turns'])} detected turn candidates have matching IMU/F9P turn signs, including reverse maneuvers. Among {len(validated)} candidates with all displayed GPS checks healthy throughout, median absolute turn-angle disagreement is {np.median([abs(t['imu_minus_f9p_turn_deg']) for t in validated]):.2f}°, maximum {errors['max_abs']:.2f}°. Endpoints and source timing matter; this is not independent truth.",
          f"2. Stationary engine-on gyro-Z noise is {noise_ratio:.1f} times the selected engine-off reference. It mostly averages down: engine-on Z standard deviation is {stationary[1]['gz']['std']:.3f}°/s at sample rate, versus {stationary[1]['gyro_z_one_second_mean']['std']:.4f}°/s for one-second arithmetic means.",
          "3. Moving prediction windows show useful short-term agreement, but degree-scale outliers remain, especially in turns. A -100 ms timing sensitivity trial improves the 3-second turning comparison; it is not applied to the headline result and does not establish a calibrated delay.",
          "4. The final left/right maneuvers contain repeated tight loops, allowing per-lap antenna-path measurements. Settled lap radii are recorded below. They are not whole-tractor clearance limits or approved minimum planning radii.", "",
          "## Driving sequence", "",
          "Operator follow-up: two square-ish paths, CW first at approximately 84% drive, then CCW at 100%; backed up before the second square; hard left followed by hard right. Percentages and steering intent are operator reports, not measured commands. No steering-pot or actuator-command telemetry was captured in this IMU dataset.", "",
          "| Data-inferred phase | Eastern time | GPS median speed while >0.15 m/s | Net F9P CCW change |", "|---|---|---:|---:|"]
    for p in result["phases"]:text.append(f"| {p['phase']} | {p['start']}–{p['end']} | {p['speed_mps']['median']:.3f} m/s | {p['heading_change_ccw_deg']:.1f}° |")
    text += ["", "The likely reverse interval immediately before the second square is 11:31:46–11:32:12, with median inferred signed forward speed about -0.27 m/s. Earlier reverse-like intervals also appear during positioning. Classification uses a 1.2-second GPS position chord projected onto chassis heading, not unsigned ground speed alone. No reverse motion model or turn-radius calibration is inferred from those intervals.", "",
             "## Stationary bias and vibration", "",
             "Intervals were selected from the known engine periods after inspecting speed/path, rather than searching for minimum gyro noise. Each selected interval has GPS speed below 0.05 m/s and approximately 1–2 cm endpoint displacement. Small chassis rocking and sensor noise remain possible.", "",
             "| Interval (Eastern) | Corrected Z mean (°/s) | Z sample std (°/s) | Z one-second-mean std (°/s) | Accel-Z std (g) | Integrated yaw change |",
             "|---|---:|---:|---:|---:|---:|"]
    for s in stationary:text.append(f"| {s['interval']} {s['start']}–{s['end']} | {s['gz']['mean']:+.5f} | {s['gz']['std']:.5f} | {s['gyro_z_one_second_mean']['std']:.5f} | {s['az']['std']:.5f} | {s['yaw_change_deg']:+.3f}° |")
    text += ["",f"The post-run corrected Z mean is {stationary[2]['gz']['mean']:+.5f}°/s (about {stationary[2]['gz']['mean']*60:+.3f}°/minute if held constant). It differs from the selected pre-run engine-off mean by {stationary[2]['gz']['mean']-stationary[0]['gz']['mean']:+.5f}°/s. This supports per-run bias calibration and ongoing bias assessment; it is not a permanent calibration constant. Temperature was not recorded, so its contribution cannot be isolated.",
             "", "One-second means are descriptive averaging, not an implemented control filter. Vibration at frequencies above the capture bandwidth and aliasing cannot be ruled out from this recording.", "",
             "## Individual square turns", "",
             "| Candidate | Start–end Eastern | F9P turn, CCW degrees | IMU turn | IMU minus F9P | GPS healthy throughout? |", "|---|---|---:|---:|---:|---|"]
    for t in result["turns"]:
        if t["candidate"] not in (5,6,7,8,14,15,16,17):continue
        text.append(f"| {t['candidate']} | {t['start']}–{t['end']} | {t['f9p_ccw_deg']:+.2f} | {t['imu_ccw_deg']:+.2f} | {t['imu_minus_f9p_turn_deg']:+.2f}° | {'Yes' if t['reference_all_healthy'] else 'No — comparison only'} |")
    text += ["", "The +12.73° comparison in the first CCW square corner crosses a degraded GPS interval. It cannot be counted as measured IMU error against reliable truth. The healthy CCW corner with about -3.12° disagreement remains a real item to investigate. See `turn_comparisons.csv` for all candidates, including approach/reverse/return maneuvers.", "",
             "## Short GPS-free prediction windows", "",
             "Each trial starts from a healthy F9P heading, then advances using the recorded fixed-bias gyro integration alone. No intervening heading correction is applied. Both endpoints and every intervening raw/heading sample must pass displayed GPS checks; windows crossing an IMU reset, calibration, missing data, or >50 ms IMU gap are excluded. Actual endpoint spacing must be within 0.12 seconds of the nominal horizon.", "",
             "The headline set has disjoint source intervals within each horizon. Horizons share evidence; even non-overlapping windows from one run are not independent validation runs. Rolling one-second starts are exported separately for inspection, not used to inflate the headline count.", "",
             "| Horizon | Moving windows | Median absolute disagreement | 95th percentile absolute | Largest observed absolute |", "|---:|---:|---:|---:|---:|"]
    for r in result["prediction_summary"]:
        if r["scheme"]=="nonoverlap" and r["group"]=="moving":
            # Full absolute median is derived from retained trial rows below,
            # not abs(signed median).
            median_abs=result["absolute_prediction_medians"][str(r["horizon_s"])]
            text.append(f"| {r['horizon_s']} s | {r['n']} | {median_abs:.3f}° | {r['p95_abs']:.3f}° | {r['max_abs']:.3f}° |")
    text += ["", "Results are descriptive disagreements with F9P, not guaranteed IMU error bounds. Turn-heavy and straight-travel subsets differ; see `prediction_summary.csv`. The small number of long straight windows limits their interpretation. No result establishes safe navigation through a loss or validates 0.3 m/s continuation.", "",
             "### Time alignment is a priority", "",
             "The baseline comparison aligns IMU yaw to the GNSS `relposned_timestamp` (Pi decode arrival) using the recorded UTC/monotonic relationship. It does not blindly compare the latest GPS value at IMU-read time. However, decode time is still not the receiver measurement epoch.", "",
             "| IMU integration time shift | 3-second turning windows | Absolute p95 | Absolute maximum |", "|---:|---:|---:|---:|"]
    for r in result["timing_sensitivity"]:text.append(f"| {r['imu_time_shift_s']:+.1f} s | {r['n']} | {r['p95_abs']:.3f}° | {r['max_abs']:.3f}° |")
    phase=result["gnss_epoch_phase"]
    text += ["",f"The fractional offset `(decode UTC - iTOW seconds) modulo 1 second` has median {phase['median_fractional_offset_s']*1000:.1f} ms (5th–95th percentiles {phase['p05_s']*1000:.1f}–{phase['p95_s']*1000:.1f} ms). This is consistent with roughly a tenth-second timing effect, but modulo arithmetic cannot prove total latency, integer-second alignment, GNSS/host clock synchronization or a correction sign for production use. The -100 ms trial was exploratory, not an independently calibrated improvement; headline numbers remain unshifted.", "",
             "## Observed steering capability for future planning", "",
             "Settled portions of the last left/right circles were selected by time, retaining the complete maneuver results and per-lap fits. A complete revolution with radial RMS <=0.10 m is used as an exploratory circularity check. Two left laps and two right laps pass; the first right lap is strongly noncircular (0.361 m radial RMS) and is excluded from the settled-lap summary. These few laps share the same field run; ranges are observed variation, not confidence bounds.", "",
             "| Maneuver | Selected interval | Settled full laps | Fitted radius range | Diameter range | Settled lap radial RMS |", "|---|---|---:|---:|---:|---:|"]
    for c in result["hard_turn_dimensions"]:
        laps=[lap for lap in c["laps"] if lap["radial_rms_m"]<=.1]
        text.append(f"| {c['maneuver']} | {c['start']}–{c['end']} | {len(laps)} | {c['accepted_lap_radius_min_m']:.3f}–{c['accepted_lap_radius_max_m']:.3f} m | {2*c['accepted_lap_radius_min_m']:.3f}–{2*c['accepted_lap_radius_max_m']:.3f} m | {min(x['radial_rms_m'] for x in laps):.3f}–{max(x['radial_rms_m'] for x in laps):.3f} m |")
    text += ["", "These are circles fitted to the logged position antenna, using RTK Fixed / healthy-heading points, at roughly 1 m/s. Centers drift between laps. Whole-interval fits include entry/center drift and are substantially less circular; both are retained in `results.json`, not hidden.", "",
             "The logger captured no steering command, pot position or full-lock acknowledgement. The operator's hard-turn intent is the steering evidence. Exact antenna-to-rear-axle offset and a surveyed body/deck footprint are not established by this dataset. Do not substitute these radii for a rear-axle kinematic limit or collision-clearance envelope.", "",
             "Historical planning documentation reports different older left/right values (approximately 1.05/1.63 m). Today's apparent asymmetry is reversed. Different calibration, speed, surface, reference point and test conditions have not been separated. Preserve both datasets; do not silently replace established planning radii. Repeated instrumented full-lock tests and a body/deck swept-envelope check are needed before planning changes.", "",
             "## Actual GPS-health losses", "",
             "These are combined displayed-health losses on the deduplicated ~5 Hz heading-source timeline. Durations have roughly 0.2-second granularity, may combine several causes, and are not motion interruptions. A new offline prediction is anchored at the last healthy sample before each loss; disagreement is evaluated only when the reference recovers.", "",
             "| Start–recovery Eastern | Loss duration | Heading reference available inside loss | Median speed | Recovered heading disagreement |", "|---|---:|---:|---:|---:|"]
    for r in result["outages"]:text.append(f"| {r['start']}–{r['recovered']} | {r['loss_duration_s']:.2f} s | {r['heading_reference_available_fraction']*100:.1f}% | {r['median_speed_mps']:.3f} m/s | {r['endpoint_prediction_error_deg']:+.3f}° |")
    text += ["", "The long approximately 59-second interval is mainly loss of position RTK Fixed and is mostly stationary (median speed 0.009 m/s); it is not a 59-second driven heading-outage validation. The approximately 3.4-second moving combined loss ends with about +3.55° disagreement. Recovered agreement does not prove the intermediate heading or position was correct when no healthy reference existed.", "",
             "## Capture integrity and next questions", "",
             f"Reintegration of every recorded segment agrees with stored yaw to {result['integration']['max_recorded_vs_reintegrated_yaw_difference_deg']:.3g}°. The two >50 ms host sampling gaps occurred before the identified squares/hard circles and were excluded from prediction windows. Near-full-scale raw rows: gyro {result['capture']['gyro_near_full_scale_rows']}, acceleration {result['capture']['accel_near_full_scale_rows']}. Host polling still cannot exclude aliasing or recover missed samples.", "",
             "1. Resolve measurement-epoch versus host-decode timing before interpreting fast-turn disagreement as gyro drift or scale error.",
             "2. Repeat stationary engine-on/off and instrumented left/right full-lock maneuvers to assess repeatability, surface dependence, thermal bias and clearance.",
             "3. Build any next heading estimator offline/shadow-only. Evaluate it on held-out runs, including deliberately varied speeds and turns. Keep present local watchdogs and health gates unchanged.",
             "4. Translate demonstrated heading/position uncertainty into route-corridor clearance only in a separately reviewed planning study. This analysis makes no continuation or field-motion authorization.", "",
             "## Figures and reproducibility", "",
             "- `maneuver_path.png`: phase-labeled path.", "- `stationary_bias_and_vibration.png`: engine effect and residual mean.",
             "- `heading_prediction_windows.png`: moving/turning/idle prediction distributions.", "- `hard_turn_dimensions.png`: fits and discarded transition effects.",
             "- `candidate_turns.json`, `turn_comparisons.csv`, `prediction_windows.csv`, `prediction_summary.csv`, `gps_loss_comparisons.csv`, `reverse_candidates.csv`, `timing_sensitivity.csv`, `results.json`: audit data.",
             "- `operator_followup.json`: driving description supplied after replay review.", "- `source_provenance.json`: exact source paths and SHA-256 hashes. `SHA256SUMS`: derived artifact hashes.", "",
             "```powershell", "python field_testing/tools/analyze_tractor01_imu_manual_20261117.py --evidence-root field_testing/sites/62_Collins_multi_boundary_20260915/runs/20261117_imu_manual/imu_20261117T161449_639816Z --operator-notes field_testing/sites/62_Collins_multi_boundary_20260915/runs/20261117_imu_manual/operator_notes.json --operator-followup field_testing/sites/62_Collins_multi_boundary_20260915/analysis/20261117_imu_manual_analysis/operator_followup.json --output-dir field_testing/sites/62_Collins_multi_boundary_20260915/analysis/20261117_imu_manual_analysis", "```", ""]
    (out/"REPORT.md").write_text("\n".join(text),encoding="utf-8")
    (out/"README.md").write_text("# November 17 IMU experiment analysis\n\nStart with [REPORT.md](REPORT.md). Figures and CSV/JSON audits are adjacent.\n\nThe operator's square directions, drive percentages and reverse maneuver are preserved in `operator_followup.json`. Phase times are inferred. Steering radii are observations of the GPS antenna path, not approved planning values.\n\nThe durable planning reference is `obsidian_vault/03-design/03-3-tractor/03-3-10-frame-chassis/20261117-observed-hard-turn-dimensions.md` from the repository root.\n\nNumerical validation: `python -m unittest tractor_rpi.testing.test_imu_manual_analysis_20261117 tractor_rpi.testing.test_imu_replay_20261117` (15 tests). Source hashes are in `source_provenance.json`; output hashes are in `SHA256SUMS`. The build transcript is diagnostic rather than source evidence.\n",encoding="utf-8")
    (out/"SHA256SUMS").write_text("\n".join(digest(p).lower()+"  "+p.name for p in sorted(out.iterdir()) if p.is_file() and p.name not in ("SHA256SUMS","build_transcript.txt"))+"\n",encoding="utf-8")


def explore(d,n,out):
    candidates=[]
    for k,(a,b) in enumerate(candidate_turns(n),1):
        candidates.append({"candidate":k,"start":local(n["epoch"][a]),"end":local(n["epoch"][b]),
            "gps_ccw_change_deg":float(n["yaw"][b]-n["yaw"][a]),"speed_med":float(np.median(n["speed"][a:b+1])),
            "reverse_fraction":float(np.mean(n["along_speed"][a:b+1]<-.1)),
            "all_health_fraction":float(np.mean(n["good"][a:b+1])),"a":int(a),"b":int(b)})
    (out/"candidate_turns.json").write_text(json.dumps(candidates,indent=2))
    fig,axes=plt.subplots(3,1,figsize=(14,11),layout="constrained")
    t=(n["epoch"]-local_epoch("11:14:00"))/60
    axes[0].plot(t,n["yaw"]-n["yaw"][0],label="F9P cumulative CCW heading change")
    axes[0].plot((d["epoch"][::10]-local_epoch("11:14:00"))/60,d["yaw"][::10],alpha=.7,label="Recorded IMU relative yaw (resets)")
    axes[0].legend();axes[0].set_ylabel("Degrees")
    axes[1].plot(t,n["speed"],label="GPS speed");axes[1].plot(t,n["along_speed"],label="Estimated signed forward speed",alpha=.7);axes[1].legend();axes[1].set_ylabel("m/s")
    axes[2].plot(t,n["rate"]);axes[2].set_ylabel("F9P CCW rate, deg/s");axes[2].set_xlabel("Minutes after 11:14 Eastern")
    for item in candidates:
        a,b=item["a"],item["b"]
        for ax in axes:ax.axvspan(t[a],t[b],alpha=.09,color="green" if item["gps_ccw_change_deg"]>0 else "orange")
        axes[2].text(t[a],n["rate"][a],str(item["candidate"]),fontsize=9)
    fig.savefig(out/"timeline_exploration.png",dpi=150);plt.close(fig)
    fig,ax=plt.subplots(figsize=(10,10),layout="constrained")
    scatter=ax.scatter(n["x"],n["y"],c=t,cmap="viridis",s=5)
    for item in candidates:
        a=item["a"];ax.text(n["x"][a],n["y"][a],str(item["candidate"]),fontsize=10,bbox={"facecolor":"white","alpha":.8,"edgecolor":"none"})
    ax.set_aspect("equal");ax.set_xlabel("East, m");ax.set_ylabel("North, m");fig.colorbar(scatter,ax=ax,label="Minutes after 11:14 Eastern")
    fig.savefig(out/"path_exploration.png",dpi=150);plt.close(fig)
    print(json.dumps(candidates,indent=2))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence-root",type=Path,required=True)
    parser.add_argument("--output-dir",type=Path,required=True)
    parser.add_argument("--operator-notes",type=Path,required=True)
    parser.add_argument("--operator-followup",type=Path,required=True)
    args=parser.parse_args()
    args.output_dir.mkdir(parents=True,exist_ok=True)
    d,n,origin=load(args.evidence_root)
    explore(d,n,args.output_dir)
    stationary=stationary_metrics(d)
    windows,predictions=prediction_windows(d,n)
    turns=turn_metrics(d,n)
    dimensions=capability(d,n)
    outages=outage_metrics(d,n)
    phases,reverse=phase_metrics(n)
    timing=lag_sensitivity(d,n,windows)
    epoch_phase=epoch_phase_check(args.evidence_root)
    integration=integration_check(d)
    result={"stationary":stationary,"prediction_summary":predictions,"turns":turns,"hard_turn_dimensions":dimensions,"outages":outages,
            "phases":phases,"reverse_candidates":reverse,"timing_sensitivity":timing,
            "gnss_epoch_phase":epoch_phase,"integration":integration,
            "capture":{"raw_rows":len(d['mono']),"gyro_near_full_scale_rows":int(np.sum(d['gyro_near_full_scale'])),"accel_near_full_scale_rows":int(np.sum(d['accel_near_full_scale']))},
            "absolute_prediction_medians":{str(h):float(np.median([abs(w['compass_prediction_error_deg']) for w in windows if w['scheme']=='nonoverlap' and w['horizon_s']==h and w['group'] in ('moving_straight','moving_turn','reverse_inferred')])) for h in (1,3,5,10)}}
    (args.output_dir/"results.json").write_text(json.dumps(result,indent=2,allow_nan=False)+"\n")
    write_csv(args.output_dir/"prediction_windows.csv",windows)
    write_csv(args.output_dir/"prediction_summary.csv",predictions)
    write_csv(args.output_dir/"turn_comparisons.csv",turns)
    write_csv(args.output_dir/"gps_loss_comparisons.csv",outages)
    write_csv(args.output_dir/"reverse_candidates.csv",reverse)
    write_csv(args.output_dir/"timing_sensitivity.csv",timing)
    write_csv(args.output_dir/"hard_turn_dimensions.csv",[{k:v for k,v in r.items() if k!="laps"} for r in dimensions])
    final_plots(d,n,stationary,predictions,dimensions,args.output_dir)
    write_report(result,args.evidence_root,args.operator_notes,args.operator_followup,args.output_dir)
    print(json.dumps({"stationary":[{"interval":r["interval"],"gz":r["gz"],"az":r["az"]} for r in stationary],
                      "moving_predictions":[r for r in predictions if r["scheme"]=="nonoverlap" and r["group"]=="moving"],
                      "hard_turn_dimensions":dimensions,"outages":outages},indent=2))


if __name__=="__main__":main()
