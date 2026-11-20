#!/usr/bin/env python3
"""Offline November 19 calibration review. Never publishes or installs calibration."""
from __future__ import annotations
import argparse
import csv
import datetime as dt
import hashlib
import json
from pathlib import Path
import sys
from collections import Counter
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
from tractor_rpi.tools.export_telemetry_run import verify


def digest(p):
    h = hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda: f.read(1048576), b''): h.update(b)
    return h.hexdigest()


def write_json(p, value):
    p.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def write_csv(p, rows):
    with p.open('w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, list(dict.fromkeys(k for r in rows for k in r)))
        w.writeheader(); w.writerows(rows)


def wrap(x): return (x + 180) % 360 - 180


def enu(lat, lon, origin):
    # WGS84 tangent-plane radii at origin, negligible truncation at this site's scale.
    a, e2 = 6378137., 6.6943799901413165e-3
    phi = np.radians(origin[0]); den = 1-e2*np.sin(phi)**2
    n = a / np.sqrt(den); m = a*(1-e2)/den**1.5
    return np.column_stack((np.radians(np.asarray(lon)-origin[1])*n*np.cos(phi),
                            np.radians(np.asarray(lat)-origin[0])*m))


def load(root):
    channels = {c: [] for c in ('imu','encoders','gnss','status','pose','events','router')}
    checks = {}
    for seg in sorted(root.glob('telemetry_*')):
        checks[seg.name] = verify(seg)
        meta = json.loads((seg/'metadata.json').read_text())
        for c in channels:
            rows = [json.loads(s) for s in (seg/(c+'.jsonl')).open(encoding='utf-8')]
            if len(rows) != meta['counts'].get(c, 0): raise ValueError('count mismatch '+c)
            channels[c].extend(rows)
    if not checks: raise ValueError('no verified segments')
    if not (root/'manual_run_20261119.tgz').is_file(): raise ValueError('required preserved archive absent')
    for arc in root.glob('*.tgz'):
        expected = arc.with_name(arc.name+'.sha256').read_text().split()[0]
        if digest(arc) != expected: raise ValueError('archive mismatch')
        if arc.name == 'manual_run_20261119.tgz' and expected != 'eb531f2c0504785f5c034073360533cef2157912524818e82a5b1bb50c0d2d19':
            raise ValueError('unexpected archive identity')
    return channels, checks


def independent_positions(imu):
    result = []
    previous = None
    for r in imu:
        p = r['payload']; age = p.get('gnss_position_age_s')
        if age is None: continue
        key = (r['source']['session'], p['monotonic_s']-age)
        if previous is not None and key[0] == previous[0] and abs(key[1]-previous[1]) <= 1e-6: continue
        previous = key
        if p.get('gnss_lat') is None or p.get('gnss_lon') is None: continue
        heading_age = p.get('gnss_heading_age_s')
        hgood = (p.get('gnss_headValid') is True and p.get('gnss_carrier') == 'fixed'
                 and p.get('gnss_relpos_valid') is True and p.get('gnss_relpos_gnss_fix_ok') is True
                 and p.get('gnss_relpos_ref_pos_miss') is False and p.get('gnss_relpos_ref_obs_miss') is False
                 and heading_age is not None and 0 <= heading_age <= .3
                 and 0 <= (p.get('gnss_relpos_heading_accuracy_deg') or 999) <= 1
                 and .8 <= (p.get('gnss_relpos_length_m') or 0) <= 1.3)
        result.append(dict(t=key[1], observed_t=p['monotonic_s'], session=key[0],
                           utc=p['utc'], lat=p['gnss_lat'], lon=p['gnss_lon'],
                           fixed=p.get('gnss_fix_quality') == 'RTK Fixed' and not p.get('gnss_fatal_error'),
                           heading_good=hgood, heading=p.get('gnss_heading_deg'),
                           heading_t=p['monotonic_s']-heading_age if heading_age is not None else None,
                           speed=p.get('gnss_speed_mps'), heading_accuracy=p.get('gnss_relpos_heading_accuracy_deg')))
    return result


def wheel_series(rows, side):
    rows = [r for r in rows if r['payload']['side'] == side]
    t = np.array([r['payload']['monotonic_s'] for r in rows])
    increments = np.array([r['payload']['delta_ticks'] for r in rows], dtype=float)
    bad = np.ones(len(rows), dtype=bool)
    for i in range(1, len(rows)):
        p, q = rows[i]['payload'], rows[i-1]['payload']
        device_dt = (p['device_ms']-q['device_ms']) & 0xffffffff
        bad[i] = not (p['contiguous'] and 0 < t[i]-t[i-1] <= .2 and 0 < device_dt <= 200
                      and rows[i]['source']['session'] == rows[i-1]['source']['session'])
    cumulative = np.cumsum(increments)
    return dict(rows=rows, t=t, n=cumulative, bad=bad, increments=increments)


def ticks_between(w, a, b):
    i = np.searchsorted(w['t'], a, side='right')-1
    j = np.searchsorted(w['t'], b, side='left')
    if i < 0 or j >= len(w['t']): return None
    if np.any(w['bad'][i+1:j+1]): return None
    # Uniform allocation only of the two boundary increments; never over a gap.
    return float(np.interp(b, w['t'], w['n'])-np.interp(a, w['t'], w['n']))


def build(root, out):
    if out == root or root in out.parents: raise ValueError('derived output inside raw evidence')
    ch, checks = load(root)
    out.mkdir(parents=True, exist_ok=True)
    session_audit = {}
    for r in ch['imu']:
        sid=r['source']['session']; p=r['payload']
        if sid not in session_audit: session_audit[sid]=dict(start_utc=p['utc'],start_mono=p['monotonic_s'],rows=0)
        session_audit[sid].update(end_utc=p['utc'],end_mono=p['monotonic_s'])
        session_audit[sid]['rows']+=1
    write_json(out/'producer_sessions.json',session_audit)
    # Explicit capture session containing the operator's 11:01-11:14 drive.
    drive_session='7295481ca757417ead5487b37862ce3e'
    drive_imu=[r for r in ch['imu'] if r['source']['session']==drive_session]
    pos = independent_positions(drive_imu)
    sessions = Counter(p['session'] for p in pos)
    if len(sessions) != 1: raise ValueError('multiple producer sessions: split explicitly before analysis')
    times = np.array([p['t'] for p in pos])
    if np.any(np.diff(times)<=0): raise ValueError('non-increasing position clock')
    origin = (pos[0]['lat'], pos[0]['lon'])
    xy = enu([p['lat'] for p in pos], [p['lon'] for p in pos], origin)
    wheels = {s: wheel_series(ch['encoders'], s) for s in 'LR'}
    # First observed IMU UTC maps source monotonic to UTC only for display.
    first = ch['imu'][0]['payload']
    offset = dt.datetime.fromisoformat(first['utc']).timestamp()-first['monotonic_s']
    utc = lambda t: dt.datetime.fromtimestamp(t+offset, dt.timezone.utc).isoformat()
    it = np.array([r['payload']['monotonic_s'] for r in drive_imu])
    iyaw = np.array([r['payload'].get('relative_yaw_ccw_deg', np.nan) for r in drive_imu])
    iseg = np.array([r['payload'].get('yaw_segment', -1) for r in drive_imu])
    st = np.array([r['monotonic_s'] for r in ch['status']])
    steer = np.array([r['payload'].get('steering',{}).get('current', np.nan) for r in ch['status']], float)
    # Predeclared link screen; consecutive qualifying links form one maneuver block.
    good = np.zeros(len(pos)-1, bool); why = []
    for i in range(len(good)):
        p,q=pos[i:i+2]; delta=q['t']-p['t']
        lo=max(0,int(np.searchsorted(times,p['t']-.5)))
        hi=min(len(times)-1,int(np.searchsorted(times,p['t']+.5)))
        span=times[hi]-times[lo]
        v=np.linalg.norm(xy[hi]-xy[lo])/span if span else 0
        reason=[]
        if not (p['fixed'] and q['fixed']): reason.append('position_not_fixed_or_fatal')
        if not (p['heading_good'] and q['heading_good']): reason.append('heading_support')
        if not .1 <= delta <= .35: reason.append('position_decode_gap')
        if not .2 <= v <= 1.8: reason.append('stationary_or_speed_outside_screen')
        if pos[lo]['heading'] is None or pos[hi]['heading'] is None or not span or abs(wrap(pos[hi]['heading']-pos[lo]['heading']))/span > 3: reason.append('turn_rate')
        # Actual steering feedback corroborates centered travel; status receipt age <= .3 s.
        k=np.searchsorted(st, p['t'], side='right')-1
        if k<0 or p['t']-st[k]>.3 or not abs(steer[k]-525)<=35: reason.append('steering_not_centered_or_stale')
        good[i]=not reason
        why.append(dict(start_mono=p['t'], end_mono=q['t'], reasons=';'.join(reason) or 'eligible_link'))
    groups=[]; start=None
    for i,v in enumerate(np.r_[good,False]):
        if v and start is None: start=i
        if not v and start is not None: groups.append((start,i)); start=None
    audits=[]; accepted=[]
    for group,(a,b) in enumerate(groups):
        # One complete block is indivisible between train/test. At most 20 s pieces.
        left=a
        while left < b:
            right=min(b, int(np.searchsorted(times, times[left]+20, side='right')-1))
            if right<=left: break
            t0,t1=times[left],times[right]; points=xy[left:right+1]
            disp=points[-1]-points[0]; distance=float(np.linalg.norm(disp))
            axis=disp/distance if distance else np.array([1.,0.])
            projection=(points-points[0])@axis
            lateral=(points-points[0])@np.array([-axis[1],axis[0]])
            # PCA is a separate fitted-track direction diagnostic, not point-distance summation.
            _,_,vt=np.linalg.svd(points-np.mean(points,axis=0),full_matrices=False)
            fitted_distance=abs(float(disp@vt[0]))
            headings=np.array([p['heading'] for p in pos[left:right+1]])
            spread=float(np.ptp(wrap(headings-headings[0])))
            motion_compass=np.degrees(np.arctan2(disp[0],disp[1]))%360
            direction=1 if abs(wrap(motion_compass-float(np.median(headings))))<30 else -1 if abs(wrap(motion_compass-float(np.median(headings))-180))<30 else 0
            ns={s:ticks_between(w,t0,t1) for s,w in wheels.items()}
            reasons=[]
            if t1-t0<5 or distance<3: reasons.append('short_block')
            if max(abs(lateral))>.15 or spread>6: reasons.append('curvature_or_lateral_residual')
            if np.min(np.diff(projection))<-.02: reasons.append('position_backtrack')
            if direction==0: reasons.append('direction_ambiguous')
            if any(n is None for n in ns.values()): reasons.append('encoder_gap_or_discontinuity')
            if any(n is not None and n*direction<=0 for n in ns.values()): reasons.append('encoder_sign_disagrees')
            row=dict(window=len(audits),block=group,start_utc=utc(t0),end_utc=utc(t1),start_mono=t0,end_mono=t1,
                     session=pos[left]['session'],duration_s=t1-t0,direction=direction,position_updates=right-left+1,
                     fixed_fraction=1.,heading_good_fraction=1.,distance_m=distance,pca_projected_distance_m=fitted_distance,
                     lateral_max_m=float(max(abs(lateral))),lateral_rms_m=float(np.sqrt(np.mean(lateral**2))),heading_range_deg=spread,
                     L_ticks=ns['L'],R_ticks=ns['R'],accepted=not reasons,reason=';'.join(reasons) or 'accepted',
                     reference_method='WGS84 local tangent endpoint displacement',boundary_allocation='linear two boundary increments')
            row['signed_heading_change_deg']=float(wrap(headings[-1]-headings[0]))
            row['mean_speed_reference_mps']=distance/(t1-t0)
            for s,w in wheels.items():
                m=(w['t']>=t0)&(w['t']<=t1)
                row[s+'_source_rows']=int(np.count_nonzero(m))
                row[s+'_invalid_rows']=int(np.count_nonzero(m&w['bad']))
                wt=w['t'][m]
                row[s+'_max_source_gap_s']=float(np.max(np.diff(wt),initial=0))
                row[s+'_encoder_events']=sum(1 for event in ch['events'] if event['payload'].get('side')==s and
                    event['payload'].get('event','').startswith('encoder_') and t0<=event['payload'].get('mono',event['monotonic_s'])<=t1)
            for s,n in ns.items():
                row[s+'_m_per_sensor_rev']=distance*direction/n*16384 if n else None
                row[s+'_provisional_m']=n*1.59593/16384 if n is not None else None
            audits.append(row)
            if not reasons: accepted.append(row)
            left=right
    # Adjacent accepted fragments on one leg share a resampling unit.
    fit_block=-1; previous=None
    for r in accepted:
        if previous is None or r['start_mono']-previous['end_mono']>5: fit_block+=1
        r['fit_block']=fit_block; previous=r
    # Chronological whole-maneuver separation: first square before 11:06 EST,
    # second-square/return after. No interleaved neighboring snapshots in validation.
    split_mono=dt.datetime.fromisoformat('2026-11-19T16:06:00+00:00').timestamp()-offset
    for r in audits: r['split']='held_out' if r['start_mono']>=split_mono else 'train'
    train=[r for r in accepted if r['split']=='train']
    test=[r for r in accepted if r['split']=='held_out']
    if len(train)<2 or not test:
        write_csv(out/'encoder_rtk_windows.csv',audits)
        write_csv(out/'position_link_exclusions.csv',why)
        raise ValueError(f'insufficient windows: train={len(train)}, test={len(test)}; inspect thresholds')
    fits={}; rng=np.random.default_rng(20261120)
    for s in 'LR':
        n=np.array([r[s+'_ticks'] for r in train]); d=np.array([r['distance_m']*r['direction'] for r in train])
        scale=float(n@d/(n@n))
        boot=[]
        trainblocks=sorted(set(r['fit_block'] for r in train))
        for _ in range(2000):
            chosen=rng.choice(trainblocks,len(trainblocks),replace=True)
            rs=[r for b in chosen for r in train if r['fit_block']==b]
            nn=np.array([r[s+'_ticks'] for r in rs]); dd=np.array([r['distance_m']*r['direction'] for r in rs])
            boot.append(float(nn@dd/(nn@nn))*16384)
        slope,intercept=np.polyfit(n,d,1)
        fits[s]=dict(metres_per_tick=scale,signed_ticks_per_metre=1/scale,metres_per_sensor_revolution=scale*16384,
                     block_bootstrap_95pct_sensor_rev_m=np.quantile(boot,[.025,.975]).tolist(),
                     free_intercept_m=float(intercept),free_slope_m_per_tick=float(slope),train_blocks=len(trainblocks))
        for r in accepted:
            predicted=r[s+'_ticks']*scale
            r[s+'_fitted_signed_m']=predicted
            r[s+'_residual_m']=predicted-r['distance_m']*r['direction']
            # Endpoint timing sensitivity is descriptive, not an estimated latency correction.
            r[s+'_timing_sensitivity_plus_100ms_m']=(ticks_between(wheels[s],r['start_mono']+.1,r['end_mono']+.1) or r[s+'_ticks'])*scale-predicted
        errors=np.array([r[s+'_residual_m'] for r in test])
        fits[s]['held_out']=dict(windows=len(test),rmse_m=float(np.sqrt(np.mean(errors**2))),max_abs_m=float(max(abs(errors))),
                                relative_error_pct=[r[s+'_residual_m']/r['distance_m']*100 for r in test])
    for r in accepted:
        r['mean_wheel_residual_m']=(r['L_residual_m']+r['R_residual_m'])/2
        r['signed_ticks_L_over_R']=r['L_ticks']/r['R_ticks']
    sensitivity=[]
    for shift in (-.2,-.1,0,.1,.2):
        for s,w in wheels.items():
            ns=[ticks_between(w,r['start_mono']+shift,r['end_mono']+shift) for r in train]
            if any(n is None for n in ns): continue
            ns=np.array(ns); ds=np.array([r['distance_m']*r['direction'] for r in train])
            scale=float(ns@ds/(ns@ns))
            errors=[]
            for r in test:
                n=ticks_between(w,r['start_mono']+shift,r['end_mono']+shift)
                if n is not None: errors.append(n*scale-r['distance_m']*r['direction'])
            sensitivity.append(dict(side=s,encoder_endpoint_shift_s=shift,refit_metres_per_sensor_rev=scale*16384,
                                    held_out_windows=len(errors),held_out_rmse_m=float(np.sqrt(np.mean(np.square(errors))))))
    write_csv(out/'timing_sensitivity.csv',sensitivity)
    # Independent position timeline, excluding repeated 20 Hz snapshots.
    write_csv(out/'independent_positions.csv',[dict(**p,east_m=float(x[0]),north_m=float(x[1])) for p,x in zip(pos,xy)])
    write_csv(out/'encoder_rtk_windows.csv',audits)
    write_csv(out/'position_link_exclusions.csv',why)
    discontinuities=[]
    for s,w in wheels.items():
        for i in np.flatnonzero(w['bad']):
            discontinuities.append(dict(side=s,mono=float(w['t'][i]),utc=w['rows'][i]['payload']['utc'],reason='first_sample_or_gap_or_device_clock_or_session_or_producer_invalid'))
    for r in ch['events']:
        if any(k in r['payload'].get('event','') for k in ('encoder_','gap','calibration','start')):
            discontinuities.append(dict(side=r['payload'].get('side'),mono=r['payload'].get('mono',r['monotonic_s']),utc=r['utc'],reason=r['payload'].get('event'),detail=json.dumps(r['payload'])))
    write_csv(out/'continuity_audit.csv',discontinuities)
    candidate=dict(status='REVIEW ONLY',reviewed_calibration=False,valid_for_control=False,
                   ticks_per_sensor_revolution=16384,mechanical_sensor_to_wheel_ratio=None,
                   reference_point='recorded GNSS position antenna; rear axle transform unmeasured',origin_wgs84=origin,
                   fit='unweighted through-origin distance versus signed ticks; complete maneuver blocks held out',
                   firmware_forward_sign={'L':1,'R':1},sensor_sign_already_applied_in_firmware={'L':-1,'R':1},
                   serials={'L':'5879620','R':'F5CB3C7E51514A4E43202020FF142006'},fits=fits)
    write_json(out/'encoder_scale_candidates.json',candidate)
    # Long healthy held-out blocks enable longer masks; turns remain geometry-confounded.
    healthy=np.array([p['fixed'] and p['heading_good'] for p in pos])
    evaluation_windows=[dict(r,stratum='calibration_straight') for r in test]
    block_start=None
    for k in range(len(times)):
        eligible=healthy[k] and times[k]>=split_mono and times[k]+offset<=dt.datetime.fromisoformat('2026-11-19T16:14:00+00:00').timestamp()
        contiguous=k==0 or .1<=times[k]-times[k-1]<=.35
        if block_start is not None and (not eligible or not contiguous or k==len(times)-1):
            end=k-1
            if times[end]-times[block_start]>=1:
                evaluation_windows.append(dict(block=1000+block_start,start_mono=float(times[block_start]),end_mono=float(times[end]),stratum='all_healthy_holdout'))
            block_start=None
        if eligible and block_start is None: block_start=k
    # Recover independent heading decode epochs too; positions need not share their clock.
    headings=[]; last_heading_t=None
    for r in drive_imu:
        p=r['payload']; age=p.get('gnss_heading_age_s')
        if age is None or p.get('gnss_heading_deg') is None: continue
        t=p['monotonic_s']-age
        if last_heading_t is None or abs(t-last_heading_t)>1e-6:
            headings.append((t,p['gnss_heading_deg'])); last_heading_t=t
    ht=np.array([h[0] for h in headings]); hd=np.degrees(np.unwrap(np.radians([h[1] for h in headings])))
    write_csv(out/'independent_headings.csv',[dict(mono=t,heading_deg=h) for t,h in headings])
    predictions=[]; coverage=[]; mask_exclusions=[]
    for horizon in (1,5,10,30):
        for mode in ('position_only','heading_only','combined'):
            attempts=Counter(); evaluated=0
            for r in evaluation_windows:
                for t0 in np.arange(r['start_mono'],r['end_mono']-horizon+.0001,horizon):
                    attempts[r['stratum']]+=1; t1=t0+horizon
                    i=np.searchsorted(times,t0); j=np.searchsorted(times,t1)
                    rejection=dict(block=r['block'],stratum=r['stratum'],mode=mode,horizon_s=horizon,start_mono=float(t0),end_mono=float(t1))
                    if j>=len(times) or times[j]-t1>.25:
                        mask_exclusions.append(dict(rejection,reason='reference_endpoint')); continue
                    ia=np.searchsorted(it,t0); ib=np.searchsorted(it,t1)
                    if ib>=len(it) or ia==ib or np.max(np.diff(it[ia:ib+1]))>.05 or np.any(iseg[ia:ib+1]!=iseg[ia]) or not np.all(np.isfinite(iyaw[ia:ib+1])):
                        mask_exclusions.append(dict(rejection,reason='imu_gap_segment_or_uncalibrated')); continue
                    # Common endpoints use reconstructed position decode epochs.
                    a,b=times[i],times[j]
                    if any(ticks_between(w,a,b) is None for w in wheels.values()):
                        mask_exclusions.append(dict(rejection,reason='wheel_gap_or_invalid')); continue
                    tt=np.r_[a,it[(it>a)&(it<b)],b]
                    yaw=np.interp(tt,it,iyaw)
                    h0=float(np.interp(a,ht,hd)); hh=h0-(yaw-yaw[0])
                    if mode=='position_only': hh=np.interp(tt,ht,hd)
                    nn=[np.interp(tt,wheels[s]['t'],wheels[s]['n'])*fits[s]['metres_per_tick'] for s in 'LR']
                    increments=(np.diff(nn[0])+np.diff(nn[1]))/2
                    travel=float(np.sum(abs(increments)))
                    if travel/(b-a)<.15:
                        mask_exclusions.append(dict(rejection,reason='below_motion_screen')); continue
                    mid=np.radians((hh[:-1]+hh[1:])/2)
                    predicted=xy[i]+np.array([np.sum(increments*np.sin(mid)),np.sum(increments*np.cos(mid))])
                    if mode=='heading_only': predicted=xy[j].copy() # independent position corrections retained
                    err=predicted-xy[j]; herror=float(wrap(hh[-1]-np.interp(b,ht,hd)))
                    direction=(xy[j]-xy[i]); norm=np.linalg.norm(direction); direction=direction/norm if norm else np.array([1.,0.])
                    turning=float(np.ptp(np.interp(tt,ht,hd)))>15
                    motion='mixed_or_reversal' if np.min(increments)<-.0001 and np.max(increments)>.0001 else 'reverse' if np.sum(increments)<0 else 'forward'
                    predictions.append(dict(block=r['block'],stratum=r['stratum'],maneuver=('turning_' if turning else 'low_turn_')+motion,horizon_s=horizon,mode=mode,start_mono=a,end_mono=b,
                        actual_horizon_s=b-a,position_error_m=float(np.linalg.norm(err)),along_error_m=float(err@direction),
                        cross_error_m=float(err@np.array([-direction[1],direction[0]])),heading_error_deg=herror,
                        reference='healthy held-out antenna path; independently decoded heading; candidate scales; unmeasured lever arm'))
                    evaluated+=1
            coverage.append(dict(horizon_s=horizon,mode=mode,attempts=dict(attempts),evaluated=evaluated))
    write_csv(out/'mask_predictions.csv',predictions)
    write_csv(out/'mask_exclusions.csv',mask_exclusions)
    masksummary=[]
    for c in coverage:
        for stratum in ('calibration_straight','all_healthy_holdout'):
            subset=[p for p in predictions if p['mode']==c['mode'] and p['horizon_s']==c['horizon_s'] and p['stratum']==stratum]
            v=dict(horizon_s=c['horizon_s'],mode=c['mode'],stratum=stratum,attempted=c['attempts'].get(stratum,0),evaluated=len(subset))
            for key in ('position_error_m','heading_error_deg'):
                values=np.abs([p[key] for p in subset])
                v[key+'_p50']=float(np.median(values)) if len(values) else None
                v[key+'_p95']=float(np.quantile(values,.95)) if len(values) else None
                v[key+'_max']=float(max(values)) if len(values) else None
            masksummary.append(v)
    write_csv(out/'mask_summary.csv',masksummary)
    maneuver_summary=[]
    for horizon in (1,5,10,30):
        for mode in ('position_only','heading_only','combined'):
            maneuvers=sorted(set(p['maneuver'] for p in predictions if p['stratum']=='all_healthy_holdout'))
            for maneuver in maneuvers:
                subset=[p for p in predictions if p['stratum']=='all_healthy_holdout' and p['horizon_s']==horizon and p['mode']==mode and p['maneuver']==maneuver]
                if subset:
                    maneuver_summary.append(dict(horizon_s=horizon,mode=mode,maneuver=maneuver,windows=len(subset),
                        position_p95_m=float(np.quantile([p['position_error_m'] for p in subset],.95)),
                        heading_abs_p95_deg=float(np.quantile(np.abs([p['heading_error_deg'] for p in subset]),.95))))
    write_csv(out/'mask_maneuver_summary.csv',maneuver_summary)
    # Once anchored versus 5-second reanchoring of the same recorded bias-corrected yaw.
    anchors=[]
    for r in evaluation_windows:
        ii=np.flatnonzero((times>=r['start_mono'])&(times<=r['end_mono']))
        im=(it>=r['start_mono'])&(it<=r['end_mono'])
        if not np.any(im) or np.max(np.diff(it[im]),initial=0)>.05 or len(set(iseg[im]))!=1: continue
        for interval in (None,5):
            anchor=int(ii[0]); errors=[]
            for k in ii:
                if interval and times[k]-times[anchor]>=interval: anchor=int(k)
                predicted=np.interp(times[anchor],ht,hd)-(np.interp(times[k],it,iyaw)-np.interp(times[anchor],it,iyaw))
                errors.append(abs(float(wrap(predicted-np.interp(times[k],ht,hd)))))
            if not np.all(np.isfinite(errors)): continue
            anchors.append(dict(block=r['block'],stratum=r['stratum'],mode='once' if interval is None else 'five_seconds',
                                duration_s=r['end_mono']-r['start_mono'],samples=len(errors),p95_abs_deg=float(np.quantile(errors,.95)),max_abs_deg=float(max(errors))))
    write_csv(out/'heading_anchor_comparison.csv',anchors)
    fig,axes=plt.subplots(2,2,figsize=(12,8))
    for s in 'LR':
        axes[0,0].scatter([r['distance_m'] for r in accepted],[r[s+'_ticks'] for r in accepted],label=s)
        axes[0,1].plot([r['window'] for r in accepted],[r[s+'_m_per_sensor_rev'] for r in accepted],'o-',label=s)
        axes[1,0].plot([r['window'] for r in accepted],[r[s+'_residual_m'] for r in accepted],'o-',label=s)
    axes[0,0].set(xlabel='RTK endpoint distance (m)',ylabel='Signed ticks')
    axes[0,1].set(xlabel='Window ID',ylabel='Metres / sensor revolution')
    axes[1,0].set(xlabel='Window ID (split in CSV)',ylabel='Fitted minus RTK (m)')
    axes[1,1].plot(xy[:,0],xy[:,1],color='lightgrey')
    for r in accepted:
        m=(times>=r['start_mono'])&(times<=r['end_mono']); axes[1,1].plot(xy[m,0],xy[m,1],label=str(r['window']))
    axes[1,1].set(xlabel='East (m)',ylabel='North (m)',aspect='equal')
    for ax in axes.flat: ax.grid(); ax.legend(fontsize=7)
    fig.tight_layout(); fig.savefig(out/'calibration_review.png',dpi=160); plt.close(fig)
    aligned=[]; jitter=[]
    for r in accepted:
        m=np.flatnonzero((times>=r['start_mono'])&(times<=r['end_mono']))
        for i in m:
            d=float(np.linalg.norm(xy[i]-xy[m[0]]))
            aligned.append(dict(window=r['window'],elapsed_s=float(times[i]-times[m[0]]),rtk_displacement_m=d,
                                **{s+'_wheel_m':float(ticks_between(wheels[s],times[m[0]],times[i])*fits[s]['metres_per_tick']) for s in 'LR'}))
    # Stationary reference screen independent of provisional wheel speeds.
    for i in range(len(times)-1):
        if pos[i]['fixed'] and pos[i]['speed'] is not None and pos[i]['speed']<.02 and np.linalg.norm(xy[i+1]-xy[i])<.02 and .1<times[i+1]-times[i]<.35:
            for s,w in wheels.items():
                n=ticks_between(w,times[i],times[i+1])
                if n is not None: jitter.append(dict(side=s,start_mono=float(times[i]),signed_ticks=n,displacement_m=n*fits[s]['metres_per_tick']))
    write_csv(out/'aligned_distance.csv',aligned); write_csv(out/'stationary_jitter.csv',jitter)
    fig,ax=plt.subplots(1,2,figsize=(12,4))
    for r in accepted:
        vals=[v for v in aligned if v['window']==r['window']]
        ax[0].plot([v['rtk_displacement_m'] for v in vals],[v['L_wheel_m']-v['rtk_displacement_m']*r['direction'] for v in vals],label=str(r['window']))
    for s in 'LR': ax[1].hist([v['signed_ticks'] for v in jitter if v['side']==s],bins=60,alpha=.5,label=s)
    ax[0].set(xlabel='RTK displacement from aligned start (m)',ylabel='Left prediction residual (m)')
    ax[1].set(xlabel='Net signed ticks / stationary position interval',ylabel='Count')
    for a in ax: a.legend(fontsize=7); a.grid()
    fig.tight_layout(); fig.savefig(out/'alignment_and_jitter.png',dpi=160); plt.close(fig)
    summary=dict(raw_counts={c:len(rows) for c,rows in ch.items()},verified_segments=checks,producer_sessions=dict(sessions),
                 independent_position_decodes=len(pos),position_decode_dt_quantiles_s=np.quantile(np.diff(times),[0,.5,.95,1]).tolist(),
                 accepted_windows=len(accepted),train_windows=len(train),held_out_windows=len(test),fits=fits,
                 direction_counts=dict(Counter(r['direction'] for r in accepted)),mask_summary=masksummary,
                 exclusion_reason_counts=dict(Counter(reason for r in why for reason in r['reasons'].split(';'))))
    write_json(out/'summary.json',summary)
    report='# Encoder / RTK calibration review — November 20, 2026\n\nREVIEW ONLY. Offline analysis; no calibration installed and no control validity granted.\n\n'
    report+=f"Verified archive and all three closed segments. Counts: {summary['raw_counts']}. The drive session contains {len(pos):,} independent position-decode updates; median spacing {np.median(np.diff(times)):.6f} s.\n\n"
    report+='| Side | Signed ticks/metre | Metres/tick | Metres/sensor revolution | Held-out RMSE | Held-out max |\n|---|---:|---:|---:|---:|---:|\n'
    for s,f in fits.items():
        report+=f"| {s} | {f['signed_ticks_per_metre']:.2f} | {f['metres_per_tick']:.9f} | {f['metres_per_sensor_revolution']:.5f} | {f['held_out']['rmse_m']:.3f} m | {f['held_out']['max_abs_m']:.3f} m |\n"
    report+=f"\nSeven forward candidate windows: three training windows on {fits['L']['train_blocks']} neighboring-leg groups, four held-out windows in the later maneuver. Fit values are descriptive, not approved calibration. Left/right fitted scale difference is {100*(fits['L']['metres_per_sensor_revolution']/fits['R']['metres_per_sensor_revolution']-1):.3f}%. The provisional 1.59593 m value overstates distance by approximately {100*(1.59593/fits['L']['metres_per_sensor_revolution']-1):.1f}% on the left and {100*(1.59593/fits['R']['metres_per_sensor_revolution']-1):.1f}% on the right relative to these candidates.\n\n"
    report+='| Horizon | Combined-loss moving trials / healthy candidates | Position median / p95 / max (m) | Heading abs median / p95 / max (degrees) |\n|---|---:|---|---|\n'
    for v in masksummary:
        if v['mode']=='combined' and v['stratum']=='all_healthy_holdout':
            report+=f"| {v['horizon_s']} s | {v['evaluated']} / {v['attempted']} | "+' / '.join(f"{v['position_error_m_'+q]:.3f}" for q in ('p50','p95','max'))+' | '+' / '.join(f"{v['heading_error_deg_'+q]:.3f}" for q in ('p50','p95','max'))+' |\n'
    report+='\nThese are empirical disagreements with healthy GNSS at host decode alignment, including turns with unmeasured lever arms. They are not a validated prediction interval or field-motion envelope.\n\n![Calibration windows and residuals](calibration_review.png)\n\n![Alignment and stationary jitter](alignment_and_jitter.png)\n\n'
    longblocks=sorted({a['block'] for a in anchors if a['stratum']=='all_healthy_holdout'},
        key=lambda b:max(a['duration_s'] for a in anchors if a['block']==b),reverse=True)[:2]
    report+='One-time versus periodic heading anchoring on the two longest healthy held-out blocks:\n\n| Block duration | Anchor policy | Heading abs p95 | Maximum |\n|---|---|---:|---:|\n'
    for b in longblocks:
        for a in anchors:
            if a['block']==b:
                report+=f"| {a['duration_s']:.1f} s | {a['mode']} | {a['p95_abs_deg']:.2f} degrees | {a['max_abs_deg']:.2f} degrees |\n"
    report+='\nPeriodic anchors use available healthy GNSS; they are not permitted inside a combined-loss mask.\n\n'
    template=Path(__file__).with_name('encoder_rtk_review_20261120_methods.md')
    report+=template.read_text(encoding='utf-8')
    (out/'REPORT.md').write_text(report,encoding='utf-8')
    sourcefiles=[p for p in root.rglob('*') if p.is_file()]+[Path(__file__)]+[REPO/p for p in (
        'tractor_rpi/position_model.py','tractor_rpi/position_gnss_protocol.py','tractor_rpi/calc_publish_position.py',
        'tractor_rpi/telemetry_recorder.py','tractor_rpi/tools/export_telemetry_run.py','tractor_rpi/imu_sensor.py',
        'tractor_rpi/testing/encoders/wheel_encoder_status_20261117_reference.md',
        'tractor_rpi/pure-pursuit/pure_pursuit_controller_20261022.py')]
    sourcefiles += [template,Path(__file__).with_name('test_analyze_encoder_rtk_20261120.py')]
    sourcefiles += [REPO/p for p in ('AGENTS.md','field_testing/TRACTOR01_CURRENT_FIELD_CONTEXT.md',
        'obsidian_vault/02-testing/20261120-pose-fusion-context.md',
        'field_testing/sites/62_Collins_multi_boundary_20260915/analysis/20261117_imu_manual_analysis/REPORT.md',
        'field_testing/sites/62_Collins_multi_boundary_20260915/analysis/20261119_manual_sensor_replay/REPORT.md',
        'obsidian_vault/03-design/03-3-tractor/03-3-10-frame-chassis/Brenham Tractor Dimensions.md')]
    write_json(out/'source_sha256.json',[dict(path=str(p.resolve()),sha256=digest(p)) for p in sourcefiles])
    (out/'SHA256SUMS').write_text(''.join(digest(p)+'  '+p.name+'\n' for p in sorted(out.iterdir()) if p.is_file() and p.name!='SHA256SUMS'),encoding='utf-8')
    print(json.dumps({k:v for k,v in summary.items() if k!='mask_summary'},indent=2))
    return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence-root',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path,required=True)
    args=parser.parse_args(); build(args.evidence_root.resolve(),args.output_dir.resolve())
