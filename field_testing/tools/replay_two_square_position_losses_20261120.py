#!/usr/bin/env python3
"""Offline encoder/gyro prediction across actual two-square position losses."""
import argparse
import datetime as dt
import json
from pathlib import Path
import numpy as np
import analyze_encoder_rtk_20261120 as support

REPO=Path(__file__).resolve().parents[2]
SESSION='7295481ca757417ead5487b37862ce3e'


def build(root,fit_file,out):
    channels,checks=support.load(root)
    fits=json.loads(fit_file.read_text())['scales']
    imu=[r for r in channels['imu'] if r['source']['session']==SESSION]
    p=support.independent_positions(imu)
    times=np.array([r['t'] for r in p])
    offset=float(np.median([dt.datetime.fromisoformat(r['payload']['utc']).timestamp()-r['payload']['monotonic_s'] for r in imu]))
    clock=lambda s:dt.datetime.fromisoformat('2026-11-19T'+s+'-05:00').timestamp()-offset
    stamp=lambda t:dt.datetime.fromtimestamp(t+offset,dt.timezone(dt.timedelta(hours=-5))).isoformat(timespec='milliseconds')
    origin=(p[0]['lat'],p[0]['lon'])
    xy=support.enu([r['lat'] for r in p],[r['lon'] for r in p],origin)
    it=np.array([r['payload']['monotonic_s'] for r in imu])
    iy=np.array([r['payload'].get('relative_yaw_ccw_deg',np.nan) for r in imu])
    segments=np.array([r['payload'].get('yaw_segment',-1) for r in imu])
    wheels={s:support.wheel_series(channels['encoders'],s) for s in 'LR'}

    def predict(i,j,kind):
        a,b=times[i],times[j]
        row=dict(kind=kind,start_est=stamp(a),recovery_or_end_est=stamp(b),duration_s=float(b-a),
                 start_mono=float(a),end_mono=float(b),status='not_evaluated')
        anchors=[r for r in p[:i+1] if r['heading_good'] and r['heading_t'] is not None and r['heading_t']<=a]
        if not anchors:return dict(row,status='no_healthy_initial_heading')
        anchor=anchors[-1]; htime=anchor['heading_t']
        im=(it>=htime)&(it<=b)
        if a-htime>1 or not np.any(im) or not np.all(np.isfinite(iy[im])) or len(set(segments[im]))!=1 or np.max(np.diff(it[im]),initial=0)>.05:
            return dict(row,status='heading_anchor_or_imu_continuity_failure')
        ns={s:support.ticks_between(w,a,b) for s,w in wheels.items()}
        if any(v is None for v in ns.values()):return dict(row,status='wheel_continuity_failure')
        tt=np.unique(np.r_[a,it[(it>a)&(it<b)],b])
        yaw=np.interp(tt,it,iy)
        heading=anchor['heading']-(yaw-float(np.interp(htime,it,iy)))
        distances=[np.interp(tt,wheels[s]['t'],wheels[s]['n'])/fits[s]['ticks_per_metre'] for s in 'LR']
        ds=(np.diff(distances[0])+np.diff(distances[1]))/2
        angle=np.radians((heading[:-1]+heading[1:])/2)
        delta=np.array([float(np.sum(ds*np.sin(angle))),float(np.sum(ds*np.cos(angle)))])
        predicted=xy[i]+delta; error=predicted-xy[j]
        h_error=None
        if p[j]['heading_good'] and p[j]['heading_t'] is not None and p[j]['heading_t']>=a:
            target_t=p[j]['heading_t']
            predicted_heading=anchor['heading']-(float(np.interp(target_t,it,iy))-float(np.interp(htime,it,iy)))
            h_error=float(support.wrap(predicted_heading-p[j]['heading']))
        return dict(row,status='evaluated',L_ticks=ns['L'],R_ticks=ns['R'],
                    mean_wheel_signed_m=float(np.sum(ds)),reference_endpoint_displacement_m=float(np.linalg.norm(xy[j]-xy[i])),
                    predicted_east_m=float(predicted[0]),predicted_north_m=float(predicted[1]),
                    reference_east_m=float(xy[j,0]),reference_north_m=float(xy[j,1]),
                    recovery_position_disagreement_m=float(np.linalg.norm(error)),east_error_m=float(error[0]),north_error_m=float(error[1]),
                    recovery_heading_disagreement_deg=h_error,
                    initial_heading_decode_est=stamp(htime),initial_heading_age_s=float(a-htime),
                    correction_policy='No GNSS position or heading corrections inside the prediction interval')

    episodes=[]; active=None
    # Bound the inventory to the two-square review span, not the later hard-turn run.
    for k in range(1,len(p)):
        if not clock('11:04:47')<=times[k]<=clock('11:08:35'):continue
        if not p[k]['fixed'] and active is None:active=k
        elif p[k]['fixed'] and active is not None:
            start=active;row=predict(start-1,k,'actual_position_non_fixed')
            row.update(first_non_fixed_est=stamp(times[start]),last_non_fixed_est=stamp(times[k-1]),non_fixed_samples=k-start)
            episodes.append(row);active=None
    if active is not None:episodes.append(dict(kind='actual_position_non_fixed',status='no_fixed_recovery_within_review_span',first_non_fixed_est=stamp(times[active])))
    masks=[]
    for start,horizon in [('11:07:35',1),('11:07:40',3),('11:07:45',5)]:
        i=int(np.searchsorted(times,clock(start)));j=int(np.searchsorted(times,times[i]+horizon))
        if not all(r['fixed'] and r['heading_good'] for r in p[i:j+1]):
            masks.append(dict(kind='healthy_combined_mask',requested_start_est=start,horizon_s=horizon,status='reference_not_healthy'))
        else:masks.append(dict(predict(i,j,'healthy_combined_mask'),nominal_horizon_s=horizon))
    out.mkdir(parents=True,exist_ok=True)
    result=dict(status='OFFLINE DEVELOPMENT CHECK; NO CONTROL AUTHORITY',
                scales={s:fits[s]['ticks_per_metre'] for s in 'LR'},actual_losses=episodes,healthy_masks=masks,
                reference_point='Recorded position antenna; no surveyed rear-axle transform',verified_segments=checks,
                limitation='Actual-loss interior position is untrusted; only recovery disagreement is measurable. Calibration and evaluation share this run, including section 8.',
                fit_sha256=support.digest(fit_file),analyzer_sha256=support.digest(Path(__file__)),helper_sha256=support.digest(Path(support.__file__)))
    support.write_json(out/'replay_results.json',result)
    report='# Two-square real position-loss replay\n\nOffline development check, November 20, 2026. Uses the all-eight operator-selected scales; no runtime changes.\n\n'
    report+='Prediction starts at the last Fixed position before a non-Fixed episode and ends at the first recovered Fixed position. Heading is anchored to the last healthy independent heading decode before prediction, then advanced with recorded fixed-bias gyro yaw. Both signed wheel streams are integrated over common host-time intervals with midpoint heading. No GNSS position or heading correction is applied inside a prediction interval, even if heading remains healthy. This is a conservative no-correction comparison, not the full proposed selective-correction estimator.\n\n'
    report+='| Last Fixed / prediction start (EST) | First non-Fixed | First recovered Fixed (EST) | Prediction duration | Recovery disagreement |\n|---|---|---|---:|---:|\n'
    for r in episodes:
        if r['status']=='evaluated':report+=f"| {r['start_est'][11:23]} | {r['first_non_fixed_est'][11:23]} | {r['recovery_or_end_est'][11:23]} | {r['duration_s']:.3f} s | {r['recovery_position_disagreement_m']:.3f} m |\n"
        else:report+=f"| — | {r.get('first_non_fixed_est','—')} | — | — | {r['status']} |\n"
    report+='\nThese durations include the last-good-to-first-bad and last-bad-to-first-good sampling brackets; they are not exact receiver outage durations. See JSON for first/last non-Fixed decode times and sample counts. Position non-Fixed episodes and heading-only losses are different; this table inventories position loss only.\n\n'
    report+='## Nearby healthy combined-loss masks\n\n| Start (EST) | Actual prediction duration | Endpoint disagreement |\n|---|---:|---:|\n'
    for r in masks:
        if r['status']=='evaluated':report+=f"| {r['start_est'][11:23]} | {r['duration_s']:.3f} s | {r['recovery_position_disagreement_m']:.3f} m |\n"
        else:report+=f"| {r.get('requested_start_est','—')} | — | {r['status']} |\n"
    report+='\nReal losses let us check that integration continues and compare against recovered Fixed GPS before a correction is applied. They cannot reveal the true path error while GPS is non-Fixed. Healthy masks retain a trustworthy reference throughout; this first check reports endpoint disagreement, with full trajectory scoring still to implement.\n\n'
    report+='All results are same-run development checks. The scales use all eight selected sections; section 8 overlaps actual position loss, so recovery closure is partly calibration-dependent and is not independent validation. Recorded antenna positions are compared against a wheel/heading prediction without measured lever-arm correction, especially important in turns. The estimator is an offline calculation, not a commissioned production-model implementation. Raw evidence, transport, safety watchdogs and motion authorization remain unchanged.\n\n'
    report+='Reproduce from the repository root:\n\n```powershell\npython field_testing/tools/replay_two_square_position_losses_20261120.py --evidence-root field_testing/sites/62_Collins_multi_boundary_20260915/runs/20261119_manual --fit field_testing/sites/62_Collins_multi_boundary_20260915/analysis/20261120_user_selected_encoder_fit/fit_results.json --output-dir field_testing/sites/62_Collins_multi_boundary_20260915/analysis/20261120_two_square_actual_losses\n```\n'
    (out/'REPORT.md').write_text(report,encoding='utf-8')
    (out/'SHA256SUMS').write_text(''.join(support.digest(p)+'  '+p.name+'\n' for p in sorted(out.iterdir()) if p.is_file() and p.name!='SHA256SUMS'))
    print(json.dumps(result,indent=2))

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence-root',type=Path,required=True);parser.add_argument('--fit',type=Path,required=True);parser.add_argument('--output-dir',type=Path,required=True)
    args=parser.parse_args();build(args.evidence_root.resolve(),args.fit.resolve(),args.output_dir.resolve())
