#!/usr/bin/env python3
"""November 20 review of Al's selected November 19 circles; offline only."""
import argparse
import datetime as dt
import json
import math
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import analyze_encoder_rtk_20261120 as shared

REPO=Path(__file__).resolve().parents[2]
WINDOWS=[('right',1,'11:09:48','11:09:55'),('right',2,'11:09:55','11:10:03'),('right',3,'11:10:03','11:10:10'),
         ('left',1,'11:11:19','11:11:27'),('left',2,'11:11:27','11:11:34'),('left',3,'11:11:34','11:11:42'),('left',4,'11:11:42','11:11:49')]

def circle(points):
    """Algebraic initialization followed by geometric radial least squares."""
    centre0=np.mean(points,axis=0);q=points-centre0
    solution,_,rank,_=np.linalg.lstsq(np.column_stack((2*q,np.ones(len(q)))),np.sum(q*q,axis=1),rcond=None)
    if rank<3:raise ValueError('degenerate circle')
    centre=solution[:2]+centre0
    radius=math.sqrt(max(0.,solution[2]+float(solution[:2]@solution[:2])))
    initial=radius
    for _ in range(30):
        delta=centre-points;norm=np.linalg.norm(delta,axis=1)
        if np.min(norm)<=1e-9:raise ValueError('point coincides with centre')
        jac=np.column_stack((delta/norm[:,None],-np.ones(len(points))))
        step=np.linalg.lstsq(jac,-(norm-radius),rcond=None)[0]
        centre+=step[:2];radius+=step[2]
        if np.linalg.norm(step)<1e-10:break
    residual=np.linalg.norm(points-centre,axis=1)-radius
    angles=np.unwrap(np.arctan2(points[:,1]-centre[1],points[:,0]-centre[0]))
    sorted_angles=np.sort(np.arctan2(points[:,1]-centre[1],points[:,0]-centre[0])%(2*np.pi))
    missing=float(np.max(np.diff(np.r_[sorted_angles,sorted_angles[0]+2*np.pi])))
    return dict(radius_m=float(radius),diameter_m=float(2*radius),centre_east_m=float(centre[0]),centre_north_m=float(centre[1]),
        radial_rms_m=float(np.sqrt(np.mean(residual**2))),radial_max_abs_m=float(np.max(abs(residual))),
        radial_p95_abs_m=float(np.quantile(abs(residual),.95)),initial_algebraic_radius_m=float(initial),
        fixed_points=len(points),fixed_arc_net_ccw_deg=float(np.degrees(angles[-1]-angles[0])),
        angular_coverage_deg=360-float(np.degrees(missing)),largest_unsampled_angle_deg=float(np.degrees(missing)))

def stats(v):
    return dict(mean=float(np.mean(v)),median=float(np.median(v)),std=float(np.std(v)),
                min=float(np.min(v)),max=float(np.max(v)),p05=float(np.quantile(v,.05)),p95=float(np.quantile(v,.95)))

def load_for_turns(root):
    """Verify the full capture while retaining only the hard-turn time range."""
    channels={c:[] for c in ('imu','encoders','status')};checks={}
    archive=root/'manual_run_20261119.tgz'
    if shared.digest(archive)!='eb531f2c0504785f5c034073360533cef2157912524818e82a5b1bb50c0d2d19':raise ValueError('archive mismatch')
    for segment in sorted(root.glob('telemetry_*')):
        checks[segment.name]=shared.verify(segment)
        meta=json.loads((segment/'metadata.json').read_text())
        for channel in ('imu','encoders','gnss','status','pose','events','router'):
            count=0
            for line in (segment/(channel+'.jsonl')).open(encoding='utf-8'):
                count+=1
                if channel in channels:
                    r=json.loads(line)
                    if '2026-11-19T16:09:44'<=r['utc']<='2026-11-19T16:11:53':channels[channel].append(r)
            if count!=meta['counts'].get(channel,0):raise ValueError('channel count mismatch')
    return channels,checks

def build(root,out):
    channels,checks=load_for_turns(root)
    imu=[r['payload'] for r in channels['imu'] if r['source']['session']=='7295481ca757417ead5487b37862ce3e']
    wrappers=[r for r in channels['imu'] if r['source']['session']=='7295481ca757417ead5487b37862ce3e']
    pos=shared.independent_positions(wrappers)
    pt=np.array([p['t'] for p in pos]);it=np.array([p['monotonic_s'] for p in imu])
    offset=float(np.median([dt.datetime.fromisoformat(p['utc']).timestamp()-p['monotonic_s'] for p in imu]))
    mono=lambda s:dt.datetime.fromisoformat('2026-11-19T'+s+'-05:00').timestamp()-offset
    xy=shared.enu([p['lat'] for p in pos],[p['lon'] for p in pos],(pos[0]['lat'],pos[0]['lon']))
    fixed=np.array([p['fixed'] for p in pos]);results=[];positions=[];imu_audit=[]
    fitfile=REPO/'field_testing/sites/62_Collins_multi_boundary_20260915/analysis/20261120_user_selected_encoder_fit/fit_results.json'
    scales=json.loads(fitfile.read_text())['scales']
    wheels={s:shared.wheel_series(channels['encoders'],s) for s in 'LR'}
    fig,axes=plt.subplots(2,4,figsize=(17,9));figrate,axrate=plt.subplots(2,1,figsize=(13,7))
    for direction,num,start,end in WINDOWS:
        ax=axes[0 if direction=='right' else 1,num-1]
        record=dict(direction=direction,circle=num,start_est=start,end_est=end)
        a,b=mono(start),mono(end)
        if b<=a:
            record.update(status='missing_corrected_time_boundary',reason='User-supplied start equals end; no radius or IMU statistics calculated.')
            results.append(record);ax.text(.5,.5,'Left circle 1\nStart equals end\nAwaiting corrected time',ha='center',va='center',transform=ax.transAxes);continue
        pm=(pt>=a)&(pt<=b);good=pm&fixed;im=(it>=a)&(it<=b)
        selected=[p for p,m in zip(imu,im) if m]
        validpts=xy[good];record.update(duration_s=b-a,position_decodes=int(np.count_nonzero(pm)),
            fixed_fraction=float(np.count_nonzero(good)/np.count_nonzero(pm)),imu_rows=len(selected),
            max_imu_gap_s=float(np.max(np.diff(it[im]),initial=0)),max_position_decode_gap_s=float(np.max(np.diff(pt[pm]),initial=0)))
        if len(validpts)>=6:
            record['circle_fit']=circle(validpts);record['status']='calculated'
            c=record['circle_fit'];theta=np.linspace(0,2*np.pi,300)
            ax.plot(c['centre_east_m']+c['radius_m']*np.cos(theta),c['centre_north_m']+c['radius_m']*np.sin(theta),'--',color='tab:blue')
        else:record['status']='insufficient_fixed_points'
        ax.plot(xy[pm,0],xy[pm,1],color='lightgrey',label='All recorded positions')
        ax.scatter(xy[good,0],xy[good,1],s=12,color='tab:green',label='Fixed fit points')
        ax.scatter(xy[pm&~fixed,0],xy[pm&~fixed,1],marker='x',s=18,color='tab:orange',label='Non-Fixed; excluded')
        yaw=np.array([p.get('relative_yaw_ccw_deg',np.nan) for p in imu])
        allgyro=np.array([p.get('gyro_z_corrected_dps',np.nan) for p in imu])
        tt=np.r_[a,it[(it>a)&(it<b)],b]
        rates=np.interp(tt,it,allgyro)
        yaw_delta=float(np.interp(b,it,yaw)-np.interp(a,it,yaw))
        contiguous=bool(record['max_imu_gap_s']<=.05 and len({p.get('yaw_segment') for p in selected})==1 and np.all(np.isfinite(rates)))
        record['imu_continuous']=contiguous
        record['gyro_z_corrected_dps']=stats([p['gyro_z_corrected_dps'] for p in selected])
        record['gyro_z_raw_dps']=stats([p['gyro_z_dps'] for p in selected])
        record['integrated_ccw_deg']=yaw_delta if contiguous else None
        record['mean_turn_rate_ccw_dps']=yaw_delta/(b-a) if contiguous else None
        record['turns_from_gyro']=yaw_delta/360 if contiguous else None
        record['seconds_per_360_at_mean_rate']=360/abs(yaw_delta/(b-a)) if contiguous and abs(yaw_delta)>0 else None
        record['gyro_other_axes_dps']={axis:stats([p['gyro_'+axis+'_corrected_dps'] for p in selected]) for axis in ('x','y')}
        record['acceleration_g']={axis:stats([p['accel_'+axis+'_g'] for p in selected]) for axis in ('x','y','z')}
        record['acceleration_magnitude_g']=stats([math.sqrt(sum(p['accel_'+axis+'_g']**2 for axis in ('x','y','z'))) for p in selected])
        # Time-weight reported RMC speed; repeated snapshots are not extra observations.
        speeds=[];last=None
        for p in imu:
            stamp=p.get('gnss_speed_timestamp')
            if stamp and stamp!=last and p.get('gnss_speed_mps') is not None:
                speeds.append((dt.datetime.fromisoformat(stamp).timestamp()-offset,p['gnss_speed_mps']));last=stamp
        area=duration=0.
        for (x,v),(y,_) in zip(speeds,speeds[1:]):
            overlap=max(0.,min(b,y)-max(a,x))
            if overlap>0 and y-x<=.5:area+=v*overlap;duration+=overlap
        record['reported_ground_speed_mean_mps']=area/duration if duration else None
        record['reported_speed_coverage_fraction']=duration/(b-a)
        record['gps_speed_over_gyro_radius_m']=(area/duration)/abs(math.radians(yaw_delta/(b-a))) if duration and contiguous and abs(yaw_delta)>0 else None
        ticks={s:shared.ticks_between(w,a,b) for s,w in wheels.items()}
        if all(v is not None for v in ticks.values()):
            d={s:ticks[s]/scales[s]['ticks_per_metre'] for s in 'LR'}
            record['wheel_travel_m']=d
            record['wheel_mean_speed_mps']=(d['L']+d['R'])/2/(b-a)
            record['mean_wheel_over_gyro_radius_m']=(d['L']+d['R'])/2/abs(math.radians(yaw_delta)) if contiguous and abs(yaw_delta)>0 else None
        steering=[r['payload']['steering']['current'] for r in channels['status'] if a<=r['monotonic_s']<=b and r['payload'].get('steering',{}).get('current') is not None]
        record['steering_pot']=stats(steering) if steering else None
        flags=[]
        if record['fixed_fraction']<1:flags.append('Non-Fixed positions excluded; fit has incomplete source coverage')
        if not contiguous:flags.append('IMU continuity failed; integrated rotation unavailable')
        if record.get('circle_fit',{}).get('angular_coverage_deg',0)<300:flags.append('Limited sampled circle arc; radius less well constrained')
        if record.get('circle_fit',{}).get('radial_rms_m',0)>.10:flags.append('Noncircular/drifting trace: radial RMS >0.10 m')
        if contiguous and abs(yaw_delta)<300:flags.append('Less than 300 degrees integrated rotation in selected interval')
        record['flags']=flags
        results.append(record)
        for k in np.flatnonzero(pm):positions.append(dict(direction=direction,circle=num,mono=float(pt[k]),east_m=float(xy[k,0]),north_m=float(xy[k,1]),fixed=bool(fixed[k])))
        for p in selected:imu_audit.append(dict(direction=direction,circle=num,mono=p['monotonic_s'],gyro_z_corrected_dps=p['gyro_z_corrected_dps'],gyro_x_corrected_dps=p['gyro_x_corrected_dps'],gyro_y_corrected_dps=p['gyro_y_corrected_dps'],accel_x_g=p['accel_x_g'],accel_y_g=p['accel_y_g'],accel_z_g=p['accel_z_g']))
        label=f"{direction.title()} {num}: {start}–{end}\n"
        if 'circle_fit' in record:label+=f"r={record['circle_fit']['radius_m']:.3f} m; Fixed={record['fixed_fraction']:.0%}; RMS={record['circle_fit']['radial_rms_m']:.3f} m"
        ax.set_title(label,fontsize=9);ax.set_aspect('equal');ax.grid();ax.set_xlabel('East (m)');ax.set_ylabel('North (m)')
        axrate[0 if direction=='right' else 1].plot(it[im]-a,[p['gyro_z_corrected_dps'] for p in selected],label=f'Circle {num}')
    axes[0,3].axis('off');axes[0,3].text(0,.9,'November 19 manual hard turns\nGPS position-antenna path\nGreen: independent Fixed positions\nOrange: non-Fixed, excluded\nBlue: geometric circle fit\n\nTimes supplied by Al, EST',va='top')
    for ax,direction in zip(axrate,('Right','Left')):ax.set(title=f'{direction} circles: corrected gyro Z',xlabel='Seconds from selected start',ylabel='CCW-positive degrees/s');ax.grid();ax.legend()
    out.mkdir(parents=True,exist_ok=True)
    fig.tight_layout();fig.savefig(out/'circle_fits.png',dpi=150);plt.close(fig)
    figrate.tight_layout();figrate.savefig(out/'imu_turn_rates.png',dpi=150);plt.close(figrate)
    shared.write_json(out/'results.json',dict(date='2026-11-19',timezone='EST',origin_wgs84=[pos[0]['lat'],pos[0]['lon']],verified_segments=checks,circles=results))
    shared.write_json(out/'position_samples.json',positions);shared.write_json(out/'imu_samples.json',imu_audit)
    report='# November 19 selected hard-turn circles and IMU\n\nAnalysis dated November 20, 2026. Observed antenna-path radii, not a commissioned planning or swept-deck limit. All times EST on November 19.\n\n'
    report+='| Direction / circle | Selected start–end | Fixed fixes | Radius | Diameter | Radial RMS | IMU mean turn rate, CCW + | IMU rotation |\n|---|---|---:|---:|---:|---:|---:|---:|\n'
    for r in results:
        if 'circle_fit' not in r:report+=f"| {r['direction']} {r['circle']} | {r['start_est']}–{r['end_est']} | — | unavailable | — | — | — | — |\n";continue
        c=r['circle_fit'];report+=f"| {r['direction']} {r['circle']} | {r['start_est']}–{r['end_est']} | {c['fixed_points']}/{r['position_decodes']} | {c['radius_m']:.3f} m | {c['diameter_m']:.3f} m | {c['radial_rms_m']:.3f} m | {r['mean_turn_rate_ccw_dps']:.2f} deg/s | {r['integrated_ccw_deg']:.1f} deg |\n"
    report+='\n## Additional IMU and motion observations\n\n| Circle | Gyro Z p05–p95 (deg/s, CCW +) | Acceleration magnitude mean / std (g) | Reported GPS mean speed | Steering pot median | GPS speed / gyro radius |\n|---|---:|---:|---:|---:|---:|\n'
    for r in results:
        if 'circle_fit' not in r:continue
        z=r['gyro_z_corrected_dps'];g=r['acceleration_magnitude_g']
        report+=f"| {r['direction']} {r['circle']} | {z['p05']:.2f} to {z['p95']:.2f} | {g['mean']:.3f} / {g['std']:.3f} | {r['reported_ground_speed_mean_mps']:.3f} m/s | {r['steering_pot']['median']:.0f} | {r['gps_speed_over_gyro_radius_m']:.3f} m |\n"
    report+='\nGyro X/Y standard deviations are approximately 5.6–6.7 and 3.7–4.5 deg/s respectively. Acceleration magnitude reaches approximately 1.8–2.25 g in individual samples, including gravity; these transient peaks are not sustained lateral acceleration. No IMU timing gap >50 ms or yaw-segment reset occurs in any selected interval. Reported rotations differ from 360 degrees because the selected whole-second boundaries are not exact revolution boundaries. Left circle 2 retains only about 235 degrees of sampled Fixed-position coverage and is less well constrained, even though its radial residual is small.\n'
    report+='\n![Circle fits](circle_fits.png)\n\n![IMU turn rates](imu_turn_rates.png)\n\n## Method and source\n\n'
    report+='Raw November 19 archive and all three closed segment hashes/counts are verified before analysis. Source session is 7295481ca757417ead5487b37862ce3e. Independent position host-decode times are reconstructed from IMU monotonic time minus position decode age, grouped within 1 microsecond; repeated 20 Hz snapshots are not counted as independent positions. Exact user intervals are retained, with no automatic lap resegmentation or exclusion of inconvenient circles. A zero-duration interval produces no result.\n\n'
    report+='Al corrected the first left interval to 11:11:19–11:11:27 and confirmed the second as 11:11:27–11:11:34 in this chat. These replace the zero-duration/overlapping entries in the initial screenshot.\n\n'
    report+='Only RTK Fixed, nonfatal positions inside each interval enter the fit. Position eligibility is independent of heading quality. WGS84 local tangent east/north coordinates use the origin in results.json. An algebraic least-squares circle initializes iterative geometric radial least squares; every eligible point has equal weight and no smoothing or outlier deletion is applied. Radial RMS/p95/max, sampled angular coverage, centres and initial algebraic radius are recorded to expose noncircularity and limited arcs. The 0.10 m RMS and 300-degree coverage flags are descriptive review screens, not automatic calibration acceptance. Whole selected intervals can contain partial/multiple revolutions.\n\n'
    report+='IMU uses native bias-corrected gyro Z and recorded continuous yaw. Positive yaw is CCW/left; negative is CW/right. Mean angular velocity is integrated yaw change over the exact selected duration, using interpolated boundary yaw. A >50 ms IMU gap or yaw-segment change invalidates integrated rotation. Native rate mean/median/std/min/max/p05/p95, raw gyro Z, other gyro axes and acceleration mean/noise/magnitude are retained in results.json. Acceleration includes gravity, mounting tilt and vibration; sensor X/Y are not automatically tractor lateral/longitudinal axes, so these are not direct lateral-acceleration or position measurements. Bias is not recalibrated during motion.\n\n'
    report+='The time-weighted reported GNSS speed divided by absolute IMU angular speed is a consistency check in metres; it shares GNSS timing/quality limitations and is not independent ground truth. Mean calibrated wheel travel divided by integrated angle is another path-radius estimate, associated approximately with rear-wheel-centre travel rather than the antenna. Do not equate these reference points without measured lever arms. Steering pot distributions use recorded status receipt times and support checking whether feedback was near commissioned right191/left885.\n\n## Per-circle flags\n\n'
    for r in results:report+=f"- {r['direction']} {r['circle']}: "+('; '.join(r.get('flags',[])) or r.get('reason') or 'No listed quality flag')+'.\n'
    report+='\nEarlier November 17 settled antenna radii were left 1.172–1.197 m and right 1.049–1.057 m; see obsidian_vault/03-design/03-3-tractor/03-3-10-frame-chassis/20261117-observed-hard-turn-dimensions.md. Its map projection and lap selection differ, so comparisons are observational, not a controlled tire/steering-change experiment. Current fits do not replace planning clearances or measure the entire tractor/deck footprint. No production code, firmware, services or motion authority changed.\n\n'
    report+='Rebuild:\n\n```powershell\npython field_testing/tools/analyze_selected_hard_turns_20261120.py --evidence-root field_testing/sites/62_Collins_multi_boundary_20260915/runs/20261119_manual --output-dir field_testing/sites/62_Collins_multi_boundary_20260915/analysis/20261120_selected_hard_turns\n```\n'
    (out/'REPORT.md').write_text(report,encoding='utf-8')
    sources=[Path(__file__),Path(shared.__file__),fitfile,root/'manual_run_20261119.tgz']+[p for seg in root.glob('telemetry_*') for p in seg.iterdir() if p.is_file()]
    shared.write_json(out/'source_sha256.json',[dict(path=str(p.resolve()),sha256=shared.digest(p)) for p in sources])
    (out/'SHA256SUMS').write_text(''.join(shared.digest(p)+'  '+p.name+'\n' for p in sorted(out.iterdir()) if p.is_file() and p.name!='SHA256SUMS'))
    print(json.dumps([dict(direction=r['direction'],circle=r['circle'],status=r['status'],radius_m=r.get('circle_fit',{}).get('radius_m'),mean_turn_rate_ccw_dps=r.get('mean_turn_rate_ccw_dps'),flags=r.get('flags')) for r in results],indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--evidence-root',type=Path,required=True);p.add_argument('--output-dir',type=Path,required=True)
    args=p.parse_args();build(args.evidence_root.resolve(),args.output_dir.resolve())
