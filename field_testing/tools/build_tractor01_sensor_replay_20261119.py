#!/usr/bin/env python3
"""Verified segmented-recorder replay; historical observations, no control output."""
from __future__ import annotations
import argparse
import bisect
from collections import Counter
import datetime as dt
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from tractor_rpi.export_telemetry_run import verify
import build_tractor01_imu_replay_20261117 as previous

HERE = Path(__file__).resolve().parent


def load(root, channel):
    rows = []
    for segment in sorted(root.glob('telemetry_*')):
        with (segment / (channel + '.jsonl')).open(encoding='utf-8') as handle:
            for line in handle:
                row = json.loads(line)
                payload = row['payload']
                stamp = previous.epoch(payload.get('utc', row['utc']))
                rows.append((stamp, payload))
    rows.sort(key=lambda item: item[0])
    return rows


def lookup(rows):
    times = [r[0] for r in rows]
    def at(stamp, age=.5):
        index = bisect.bisect_right(times, stamp) - 1
        return rows[index][1] if index >= 0 and stamp-times[index] <= age else {}
    return at


def build(root, output, start, end):
    if output == root or root in output.parents:
        raise ValueError('Keep derived output outside raw evidence')
    manifests = {p.name: verify(p) for p in sorted(root.glob('telemetry_*'))}
    if not manifests:
        raise ValueError('No verified segments')
    for archive in root.glob('*.tgz'):
        expected = archive.with_name(archive.name+'.sha256').read_text().split()[0]
        if hashlib.sha256(archive.read_bytes()).hexdigest() != expected:
            raise ValueError('Archive checksum mismatch')
    channels = {c: load(root, c) for c in ('imu', 'encoders', 'status', 'pose', 'events', 'gnss')}
    counts = {c: len(rows) for c, rows in channels.items()}
    # Verify the count of every consumed stream against its closed-segment metadata.
    for c, count in counts.items():
        expected = sum(json.loads((p/'metadata.json').read_text())['counts'].get(c, 0)
                       for p in root.glob('telemetry_*'))
        if expected != count:
            raise ValueError(f'{c}: metadata count mismatch')
    imu = [(t, p) for t, p in channels['imu'] if start <= t <= end]
    if not imu:
        raise ValueError('No IMU data in requested window')
    epoch0, first = imu[0]
    mono0 = first['monotonic_s']
    located = next(p for _,p in imu if p.get('gnss_lat') is not None and p.get('gnss_lon') is not None)
    origin = (located['gnss_lat'], located['gnss_lon'],
              111320*math.cos(math.radians(located['gnss_lat'])))
    wheels = {side: lookup([(t,p) for t,p in channels['encoders'] if p['side']==side]) for side in 'LR'}
    status, pose = lookup(channels['status']), lookup(channels['pose'])
    samples, gaps, episodes = [], [], []
    health_counts, active = Counter(), {}
    last_bucket = None
    pending = None
    ranges = {}
    last = None
    for stamp, payload in imu:
        row = dict(payload)
        # Direct-serial capture has no UDP socket. Reuse the earlier health helper
        # with the recorded position decode age, explicitly relabelled in the UI.
        row['gnss_socket_available'] = True
        row['gnss_packet_age_s'] = row.get('gnss_position_age_s')
        s = previous.compact(row, origin, mono0)
        if last and s['t'] <= last['t']:
            raise ValueError('Non-increasing IMU source clock')
        for key in ('fresh', 'fixed', 'heading', 'baselineOk'):
            health_counts[key] += int(s['health'][key])
            if not s['health'][key] and key not in active:
                active[key] = s['t']
            elif s['health'][key] and key in active:
                a = active.pop(key)
                episodes.append(dict(condition=key,start_s=a,end_s=s['t'],duration_s=s['t']-a))
        if last and (s['segment'] != last['segment'] or s['t']-last['t']>.05):
            gaps.append(dict(t=s['t'],dt_s=s['t']-last['t'],segment=s['segment']))
        for side in 'LR':
            wheel = wheels[side](stamp)
            s['wheel'+side] = wheel.get('reported_speed_mps')
            s['ticks'+side] = wheel.get('delta_ticks')
        st = status(stamp)
        arb = st.get('rpi_arbitrator', {})
        s.update(steer=st.get('steering',{}).get('current'),
                 actual=arb.get('actual_serial_output'),requested=arb.get('requested_mode'),
                 drive=arb.get('joined_drive') if arb.get('joined_mode')==1 else 0 if arb.get('joined_mode')==0 else None,
                 voltage=(st.get('transmission',{}).get('vin_voltage_mV',0)/1000 or None))
        s['shadowHeading'] = pose(stamp).get('pose',{}).get('heading_deg')
        bucket = math.floor(s['t']/.1)
        changed = pending and (pending['health']!=s['health'] or pending['segment']!=s['segment'])
        if pending and (bucket != last_bucket or changed):
            pending['range'] = {k:[min(v),max(v)] for k,v in ranges.items()}
            samples.append(pending)
            ranges = {}
        for k in previous.SIGNALS:
            if s[k] is not None:
                ranges.setdefault(k,[]).append(s[k])
        last_bucket, pending, last = bucket, s, s
    pending['range'] = {k:[min(v),max(v)] for k,v in ranges.items()}
    samples.append(pending)
    for key,a in active.items():
        episodes.append(dict(condition=key,start_s=a,end_s=last['t'],duration_s=last['t']-a))
    events = []
    for stamp,p in channels['events']:
        name = p.get('event','')
        if start<=stamp<=end and (name=='candidate_ten_second_stop' or 'calibration' in name or 'gap' in name or 'error' in name or 'stale' in name):
            events.append(dict(t=stamp-epoch0,category='recorded',label=name.replace('_',' ')))
    notes = [('2026-11-19T10:59:00-05:00','Engine off · manual steering check, then Pause'),
             ('2026-11-19T11:01:00-05:00','Engine running · two start attempts reported'),
             ('2026-11-19T11:14:00-05:00','Parked · engine off · Pause (approximate)')]
    for stamp,label in notes:
        t = previous.epoch(stamp)-epoch0
        events.append(dict(t=t,category='operator',label=label+' · minute precision',uncertainUntil=t+60))
    for g in gaps:
        events.append(dict(t=g['t'],category='gap',label=f"IMU timing gap {g['dt_s']*1000:.1f} ms"))
    times = [s['t'] for s in samples]
    for e in events:
        e['sample'] = min(len(samples)-1,max(0,bisect.bisect_left(times,e['t'])))
    events.sort(key=lambda e:e['t'])
    calibration = next(p['calibration'] for t,p in channels['pose'] if t>=epoch0 and (p.get('calibration') or {}).get('bias_dps'))
    metadata = json.loads((next(root.glob('telemetry_*'))/'metadata.json').read_text())
    metadata.update(calibration=calibration,yaw_segments=len({s['segment'] for s in samples}))
    sources = [p for seg in root.glob('telemetry_*') for p in seg.iterdir() if p.is_file()]
    sources += [Path(__file__), HERE/'tractor-sensor-replay-template.html',
                HERE/'build_tractor01_imu_replay_20261117.py',previous.SHARED_TEMPLATE,
                HERE/'build_tractor_mission_replays_20260919.py',
                HERE/'check_sensor_replay_ui_20261119.cjs',
                HERE.parents[1]/'tractor_rpi/export_telemetry_run.py']
    sources += list(root.glob('preflight*')) + list(root.glob('*.tgz*'))
    provenance = [dict(name=p.as_posix(),sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in sources]
    deltas = [p['dt_s'] for _,p in imu if p.get('dt_s') is not None]
    summary = dict(rawRows=len(imu),playbackSamples=len(samples),duration_s=samples[-1]['t'],
                   startEpoch=epoch0,median_dt_s=statistics.median(deltas),max_dt_s=max(deltas),
                   counts=counts,health_fraction={k:v/len(imu) for k,v in health_counts.items()},
                   verified_manifests=manifests,origin=list(origin[:2]))
    payload = dict(title='November 19, 2026 · Manual GPS / IMU / encoder replay',run=root.name,
                   samples=samples,events=events,episodes=episodes,gaps=gaps,metadata=metadata,
                   summary=summary,source=provenance)
    template = (HERE/'tractor-sensor-replay-template.html').read_text(encoding='utf-8')
    # Reuse the established shared style/geometry renderer without changing its template global.
    shared = previous.SHARED_TEMPLATE.read_text(encoding='utf-8')
    page = template.replace('__SHARED_STYLE__',shared.split('<style>',1)[1].split('</style>',1)[0])
    page = page.replace('__SHARED_GEOMETRY__',shared[shared.index('function viewport('):shared.index('function drawMap(')])
    page = page.replace('__DATA__',json.dumps(payload,separators=(',',':'),allow_nan=False).replace('<','\\u003c'))
    output.mkdir(parents=True,exist_ok=True)
    (output/'replay_20261119_manual.html').write_text(page,encoding='utf-8',newline='\n')
    (output/'summary.json').write_text(json.dumps(dict(summary=summary,gaps=gaps,episodes=episodes,source=provenance),indent=2)+'\n',encoding='utf-8')
    report = f'''# November 19 manual sensor replay

Open replay_20261119_manual.html locally; no network or tractor connection required.

Archive and all three closed segment manifests independently verified on Windows.
All consumed channel counts match closed metadata. Raw originals are preserved.
This replay includes {len(imu):,} full-rate IMU rows, displayed in 100 ms bins
with gyro/acceleration extrema and extra health/segment transition boundaries.
Timeline starts {dt.datetime.fromtimestamp(epoch0,dt.timezone.utc).isoformat()}.

GPS positions and IMU comparisons are recorded observations. Direct-serial position
decode age replaces the old standalone logger's UDP packet age in the displayed
health test. Position Fixed, heading validity/freshness/carrier/accuracy and baseline
checks follow the preceding replay; these labels do not reconstruct motion authority.
Detailed fractions and overlapping condition episodes are in summary.json.
Snapshot rates are not independent GNSS measurement rates (heading is about 5 Hz).

Wheel speeds and delta ticks use the latest preceding source record within 0.5 s;
no future samples are joined. Scale is provisional, not calibrated. Status is joined
the same way, using recorder arrival time where no source UTC exists. Drive chart
shows joined Manual-percent output, Pause zero, other/missing modes blank.
Steering is actual pot feedback. JRK VIN alone cannot determine engine state.

Heading comparison is the recorded once-anchored IMU output, not the separately
five-second-reanchored shadow pose and not a navigation estimate. The shadow pose
remains invalid for control. Timing gaps break integration; yaw across segments
cannot be subtracted. Host read intervals do not reconstruct missed sensor samples.

Operator notes: 10:59 engine off/manual steering check; 11:01 engine running after
two start attempts; approximately 11:14 parked/off/Pause. Unknown seconds are
shown as one-minute bands; the parking time has additional unspecified uncertainty.
Ten-second-stop events are automatic candidates, not confirmed maneuver boundaries.
CW/CCW squares and hard turns were reported; dimensions still require analysis.

Full source SHA-256 provenance is embedded in the HTML and summary.json.
Derived artifacts are separate from raw recordings. Nothing here changes control.
'''
    (output/'REPORT.md').write_text(report,encoding='utf-8',newline='\n')
    (output/'README.md').write_text('''# November 19 manual replay

Open `replay_20261119_manual.html`. Use Play, the timeline, event menu and chart
window controls. All data is embedded; no server or internet connection is needed.
Wheel scale is provisional. See REPORT.md for definitions and limitations.

Rebuild from the independently verified preserved raw copy:

```powershell
python field_testing/tools/build_tractor01_sensor_replay_20261119.py --evidence-root field_testing/sites/62_Collins_multi_boundary_20260915/runs/20261119_manual --output-dir field_testing/sites/62_Collins_multi_boundary_20260915/analysis/20261119_manual_sensor_replay
node field_testing/tools/check_sensor_replay_ui_20261119.cjs field_testing/sites/62_Collins_multi_boundary_20260915/analysis/20261119_manual_sensor_replay/replay_20261119_manual.html
```
''',encoding='utf-8',newline='\n')
    (output/'SHA256SUMS').write_text(''.join(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+p.name+'\n' for p in sorted(output.iterdir()) if p.is_file() and p.name!='SHA256SUMS'),encoding='utf-8',newline='\n')
    return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence-root',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--start',default='2026-11-19T10:57:00-05:00')
    parser.add_argument('--end',default='2026-11-19T11:16:00-05:00')
    args=parser.parse_args()
    print(json.dumps(build(args.evidence_root.resolve(),args.output_dir.resolve(),previous.epoch(args.start),previous.epoch(args.end)),indent=2))
