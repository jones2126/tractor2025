"""Read-only raw evidence extraction for Al's eight selected time slices."""
import argparse
import csv
import datetime as dt
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys
import numpy as np

REPO=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(REPO))
from tractor_rpi.tools.export_telemetry_run import verify

ROOT=REPO/'field_testing/sites/62_Collins_multi_boundary_20260915/runs/20261119_manual'
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output-dir',type=Path,required=True)
args=parser.parse_args()
OUT=args.output_dir.resolve()
if OUT==ROOT.resolve() or ROOT.resolve() in OUT.parents:raise ValueError('Keep extraction outside raw evidence')
OUT.mkdir(parents=True,exist_ok=True)
SESSION='7295481ca757417ead5487b37862ce3e'
SLICES=[('11:04:47','11:05:07'),('11:05:12','11:05:25'),('11:05:29','11:05:50'),('11:05:53','11:06:08'),
        ('11:07:13','11:07:28'),('11:07:32','11:07:52'),('11:07:56','11:08:12'),('11:08:17','11:08:35')]

def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()

archive=ROOT/'manual_run_20261119.tgz'
assert sha(archive)=='eb531f2c0504785f5c034073360533cef2157912524818e82a5b1bb50c0d2d19'
channels={c:[] for c in ('imu','encoders','events')}
source=[]; counts={}
for segment in sorted(ROOT.glob('telemetry_*')):
    verify(segment)
    meta=json.loads((segment/'metadata.json').read_text())
    for c in ('imu','encoders','gnss','status','pose','events','router'):
        count=0
        with (segment/(c+'.jsonl')).open() as f:
            for line in f:
                count+=1
                if c in channels:
                    r=json.loads(line)
                    if (r.get('source') or {}).get('session')==SESSION:channels[c].append(r)
        assert count==meta['counts'].get(c,0),(segment,c,count)
        counts[c]=counts.get(c,0)+count
    for p in segment.iterdir():
        if p.is_file():source.append(dict(file=str(p.relative_to(REPO)),sha256=sha(p)))
source.append(dict(file=str(archive.relative_to(REPO)),sha256=sha(archive)))
imu=channels['imu']
offsets=[dt.datetime.fromisoformat(r['payload']['utc']).timestamp()-r['payload']['monotonic_s'] for r in imu]
offset=statistics.median(offsets)
pos=[]; heads=[]; lp=lh=None
for r in imu:
    p=r['payload']; t=p['monotonic_s']
    age=p.get('gnss_position_age_s')
    if age is not None and p.get('gnss_lat') is not None:
        decode=t-age
        if lp is None or abs(decode-lp)>1e-6:
            pos.append(dict(t=decode,lat=p['gnss_lat'],lon=p['gnss_lon'],
                good=p.get('gnss_fix_quality')=='RTK Fixed' and not p.get('gnss_fatal_error'),quality=p.get('gnss_fix_quality')))
            lp=decode
    age=p.get('gnss_heading_age_s')
    if age is not None and p.get('gnss_heading_deg') is not None:
        decode=t-age
        if lh is None or abs(decode-lh)>1e-6:
            good=(p.get('gnss_headValid') is True and p.get('gnss_carrier')=='fixed' and
                p.get('gnss_relpos_valid') is True and p.get('gnss_relpos_gnss_fix_ok') is True and
                p.get('gnss_relpos_ref_pos_miss') is False and p.get('gnss_relpos_ref_obs_miss') is False and
                0 <= (p.get('gnss_relpos_heading_accuracy_deg') or 999)<=1 and .8<=(p.get('gnss_relpos_length_m') or 0)<=1.3)
            heads.append(dict(t=decode,h=p['gnss_heading_deg'],good=good,accuracy=p.get('gnss_relpos_heading_accuracy_deg')))
            lh=decode
pt=np.array([p['t'] for p in pos]); ht=np.array([p['t'] for p in heads])
assert np.all(np.diff(pt)>0) and np.all(np.diff(ht)>0)
hd=np.degrees(np.unwrap(np.radians([p['h'] for p in heads])))
lats=np.array([p['lat'] for p in pos]); lons=np.array([p['lon'] for p in pos])
phi=math.radians(pos[0]['lat']); e2=6.6943799901413165e-3; a=6378137.; den=1-e2*math.sin(phi)**2
n=a/math.sqrt(den); m=a*(1-e2)/den**1.5
xy=np.column_stack((np.radians(lons-lons[0])*n*math.cos(phi),np.radians(lats-lats[0])*m))
wheel={s:[r['payload'] for r in channels['encoders'] if r['payload']['side']==s] for s in 'LR'}
utc=lambda t:dt.datetime.fromtimestamp(t+offset,dt.timezone.utc).isoformat()
local=lambda t:dt.datetime.fromtimestamp(t+offset,dt.timezone(dt.timedelta(hours=-5))).strftime('%H:%M:%S.%f')[:-3]

def bracket(times,t):
    j=int(np.searchsorted(times,t,side='right')); i=j-1
    assert i>=0 and j<len(times)
    return i,j

def ticks(side,start,end):
    rows=wheel[side]; total=0.; whole=0; partial=0.; count=0; invalid=0; maxgap=0.; intervals=[]
    for prev,p in zip(rows,rows[1:]):
        x,y=prev['monotonic_s'],p['monotonic_s']
        overlap=max(0.,min(end,y)-max(start,x))
        if overlap<=0:continue
        gap=y-x; device=(p['device_ms']-prev['device_ms'])&0xffffffff
        valid=bool(p['contiguous'] and 0<gap<=.2 and 0<device<=200)
        fraction=overlap/gap
        ticks=p['delta_ticks']*fraction
        intervals.append(dict(side=side,interval_start_utc=utc(x),interval_end_utc=utc(y),raw_ticks=p['delta_ticks'],fraction=fraction,allocated_ticks=ticks,valid=valid))
        total+=ticks; count+=1; invalid+=not valid; maxgap=max(maxgap,gap)
        if abs(fraction-1)<1e-8:whole+=p['delta_ticks']
        else:partial+=ticks
    # Independent cumulative interpolation must agree with overlap allocation.
    tt=[p['monotonic_s'] for p in rows]; cumulative=np.cumsum([p['delta_ticks'] for p in rows])
    check=float(np.interp(end,tt,cumulative)-np.interp(start,tt,cumulative))
    assert abs(check-total)<1e-6
    assert abs(total-whole-partial)<1e-6
    events=[r['payload'] for r in channels['events'] if r['payload'].get('side')==side and
        r['payload'].get('event','').startswith('encoder_') and start<=r['payload'].get('mono',r['monotonic_s'])<=end]
    return dict(ticks=total,whole_ticks=whole,boundary_ticks=partial,intervals=count,invalid=invalid,max_gap_s=maxgap,
                event_count=len(events),first_device_ms=next(p['device_ms'] for p in rows if p['monotonic_s']>=start),
                last_device_ms=next(p['device_ms'] for p in reversed(rows) if p['monotonic_s']<=end),allocated_intervals=intervals)

result=[]; allocations=[]
for index,(begin,end) in enumerate(SLICES,1):
    start=dt.datetime.fromisoformat('2026-11-19T'+begin+'-05:00').timestamp()-offset
    finish=dt.datetime.fromisoformat('2026-11-19T'+end+'-05:00').timestamp()-offset
    pi,pj=bracket(pt,start); qi,qj=bracket(pt,finish)
    hi,hj=bracket(ht,start); ki,kj=bracket(ht,finish)
    points=pos[pi:qj+1]; heading_points=heads[hi:kj+1]
    pgood=sum(p['good'] for p in points)/len(points); hgood=sum(p['good'] for p in heading_points)/len(heading_points)
    startxy=np.array([np.interp(start,pt,xy[:,k]) for k in (0,1)])
    endxy=np.array([np.interp(finish,pt,xy[:,k]) for k in (0,1)])
    delta=endxy-startxy; distance=float(np.linalg.norm(delta)); unit=delta/distance
    inside=(pt>start)&(pt<finish)
    track=np.vstack((startxy,xy[inside],endxy))
    lateral=(track-startxy)@np.array([-unit[1],unit[0]])
    h0=float(np.interp(start,ht,hd)); h1=float(np.interp(finish,ht,hd))
    hvalues=np.r_[h0,hd[(ht>=start)&(ht<=finish)],h1]
    left=float(min(hvalues)); right=float(max(hvalues)); mean=float(np.mean(hvalues))
    L=ticks('L',start,finish); R=ticks('R',start,finish)
    for s,w in (('L',L),('R',R)):
        allocations.extend(dict(slice=index,**v) for v in w.pop('allocated_intervals'))
    flag=[]
    if pgood<1:flag.append('position not Fixed throughout')
    if hgood<1:flag.append('heading quality failed')
    if L['invalid'] or R['invalid']:flag.append('encoder discontinuity')
    if L['event_count'] or R['event_count']:flag.append('encoder event')
    if max(np.diff(pt[pi:qj+1]))>.35:flag.append('position decode gap >0.35 s')
    if right-left>6:flag.append('heading range >6 deg')
    if max(abs(lateral))>.15:flag.append('lateral deviation >0.15 m')
    result.append(dict(id=index,start=begin,end=end,duration_s=finish-start,start_mono=start,end_mono=finish,
        heading_left=left%360,heading_right=right%360,heading_range=right-left,heading_start=h0%360,heading_end=h1%360,
        heading_net=h1-h0,heading_mean=mean%360,heading_std=float(np.std(hvalues)),heading_count=len(heading_points),heading_good_fraction=hgood,
        heading_accuracy_max=max(p['accuracy'] for p in heading_points if p['accuracy'] is not None),
        lat_start=float(np.interp(start,pt,lats)),lon_start=float(np.interp(start,pt,lons)),
        lat_end=float(np.interp(finish,pt,lats)),lon_end=float(np.interp(finish,pt,lons)),
        east_change_m=float(delta[0]),north_change_m=float(delta[1]),distance_m=distance,
        lateral_max_m=float(max(abs(lateral))),lateral_rms_m=float(np.sqrt(np.mean(lateral**2))),
        position_count=len(points),position_good_fraction=pgood,position_max_gap_s=float(max(np.diff(pt[pi:qj+1]))),
        position_start_before=local(pt[pi]),position_start_after=local(pt[pj]),position_end_before=local(pt[qi]),position_end_after=local(pt[qj]),
        start_position_bracket_s=float(pt[pj]-pt[pi]),end_position_bracket_s=float(pt[qj]-pt[qi]),
        L=L,R=R,flags='; '.join(flag) or 'Fixed position; supported heading; continuous wheels',
        distance_label='RTK antenna endpoint displacement; exact-boundary interpolation'))
data=dict(date='2026-11-19',timezone='EST (UTC-05:00)',session=SESSION,counts=counts,source_hashes=source,
          utc_minus_mono_median_s=offset,utc_minus_mono_spread_s=max(offsets)-min(offsets),slices=result)
(OUT/'slices.json').write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')
with (OUT/'allocated_encoder_intervals.csv').open('w',newline='') as f:
    w=csv.DictWriter(f,list(allocations[0]));w.writeheader();w.writerows(allocations)
print(json.dumps([dict(id=r['id'],start=r['start'],end=r['end'],heading_range=r['heading_range'],distance_m=r['distance_m'],L_ticks=r['L']['ticks'],R_ticks=r['R']['ticks'],flags=r['flags']) for r in result],indent=2))
