#!/usr/bin/env python3
"""Fit empirical encoder scales to Al's eight selected November 19 sections."""
import argparse
import hashlib
import json
import math
from pathlib import Path

REPO=Path(__file__).resolve().parents[2]

def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for block in iter(lambda:f.read(1048576),b''):h.update(block)
    return h.hexdigest()

def fit(rows,side):
    """Trusted distances are predictors; minimize squared tick residuals at zero intercept."""
    return sum(r['distance_m']*r[side]['ticks'] for r in rows)/sum(r['distance_m']**2 for r in rows)

def metrics(errors):
    return dict(rmse_m=math.sqrt(sum(e*e for e in errors)/len(errors)),
                mean_error_m=sum(errors)/len(errors),max_abs_error_m=max(map(abs,errors)))

def build(source,out):
    data=json.loads(source.read_text())
    # Reverify the preserved raw evidence hashes carried by the extraction.
    for item in data['source_hashes']:
        if sha(REPO/item['file'])!=item['sha256']:raise ValueError('source mismatch: '+item['file'])
    rows=data['slices']
    if len(rows)!=8 or [r['id'] for r in rows]!=list(range(1,9)):raise ValueError('expected eight selected sections')
    for r in rows:
        if r['distance_m']<=0 or r['duration_s']<=0:raise ValueError('invalid distance/time')
        for side in 'LR':
            if r[side]['invalid'] or r[side]['event_count']:raise ValueError('invalid wheel evidence')
    scales={}; audit=[]
    for side in 'LR':
        k=fit(rows,side)
        residuals=[r[side]['ticks']/k-r['distance_m'] for r in rows]
        omitted=[]; predictions=[]
        for r in rows:
            kk=fit([q for q in rows if q['id']!=r['id']],side)
            omitted.append(kk);predictions.append(r[side]['ticks']/kk-r['distance_m'])
        first=fit(rows[:4],side)
        reverse=fit(rows[4:],side)
        chronological=[r[side]['ticks']/first-r['distance_m'] for r in rows[4:]]
        scales[side]=dict(ticks_per_metre=k,metres_per_tick=1/k,metres_per_magnet_revolution=16384/k,
            pooled_ticks_over_pooled_distance=sum(r[side]['ticks'] for r in rows)/sum(r['distance_m'] for r in rows),
            fitted_distance_residuals=metrics(residuals),leave_one_section_out=metrics(predictions),
            leave_one_section_out_scale_range=[min(omitted),max(omitted)],
            first_four_fit_ticks_per_metre=first,last_four_fit_ticks_per_metre=reverse,
            first_four_fit_last_four_validation=metrics(chronological))
        for i,r in enumerate(rows):
            audit.append(dict(section=r['id'],side=side,start_est=r['start'],end_est=r['end'],duration_s=r['duration_s'],
                rtk_distance_m=r['distance_m'],signed_ticks=r[side]['ticks'],section_ticks_per_metre=r[side]['ticks']/r['distance_m'],
                predicted_distance_m=r[side]['ticks']/k,residual_m=residuals[i],residual_pct=100*residuals[i]/r['distance_m'],
                omitted_section_scale=omitted[i],omitted_section_prediction_error_m=predictions[i],
                heading_range_deg=r['heading_range'],fixed_fraction=r['position_good_fraction'],
                heading_healthy_fraction=r['heading_good_fraction'],original_flags=r['flags']))
    total_distance=sum(r['distance_m'] for r in rows)
    result=dict(status='REVIEW ONLY — EMPIRICAL FIT; NOT INSTALLED',reviewed_calibration=False,valid_for_control=False,
        selected_sections=list(range(1,9)),total_reference_distance_m=total_distance,
        ticks_per_magnet_revolution=16384,physical_circumference_assumed=False,
        method='N_i = k_i * D; k = sum(D*N) / sum(D^2), through origin, all eight selected sections retained',
        interpretation='RTK endpoint distances accepted as travel reference for these operator-selected basically straight sections',
        source=json.loads(source.read_text())['session'],input_sha256=sha(source),scales=scales,windows=audit)
    out.mkdir(parents=True,exist_ok=True)
    (out/'selected_sections.json').write_bytes(source.read_bytes())
    (out/'fit_results.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    report='# Encoder fit using Al’s selected sections\n\nNovember 20, 2026. REVIEW ONLY; no runtime changes.\n\n'
    report+='All eight sections are retained exactly as selected. RTK endpoint displacement is accepted as the travel reference for these basically straight sections, per Al’s instruction. The 16,384 ticks per magnet revolution constant is unchanged. No physical tire circumference or gearing is inferred.\n\n'
    report+='| Side | Fitted ticks/metre | Metres/tick | Effective metres/magnet revolution | Fit RMSE | Leave-one-section-out RMSE |\n|---|---:|---:|---:|---:|---:|\n'
    for s,f in scales.items():
        report+=f"| {s} | {f['ticks_per_metre']:.3f} | {f['metres_per_tick']:.9f} | {f['metres_per_magnet_revolution']:.6f} | {f['fitted_distance_residuals']['rmse_m']:.3f} m | {f['leave_one_section_out']['rmse_m']:.3f} m |\n"
    report+=f'\nTotal reference distance: {total_distance:.3f} m across eight sections.\n\n'
    report+='Fit `ticks = ticks_per_metre * GPS_distance` through zero, separately for each side: `ticks_per_metre = SUM(distance * ticks) / SUM(distance^2)`. This treats the accepted GPS distances as predictors and minimizes squared tick residuals, rather than averaging section ratios equally. Longer sections have more influence. Signed ticks remain positive for forward; the existing firmware has already applied the left sensor sign.\n\n'
    report+='Use `distance_m = signed_ticks / ticks_per_metre` and `speed_mps = distance_m / elapsed_seconds`. The equivalent metres/magnet-revolution value can be used in an older formula, but should not be called a measured wheel circumference.\n\n'
    report+='## Per-section distance checks using the all-eight fit\n\n| Section | EST start–end | RTK distance | Left error | Right error |\n|---|---|---:|---:|---:|\n'
    for r in rows:
        a=[q for q in audit if q['section']==r['id']]
        report+=f"| {r['id']} | {r['start']}–{r['end']} | {r['distance_m']:.3f} m | {a[0]['residual_m']:+.3f} m | {a[1]['residual_m']:+.3f} m |\n"
    report+='\nErrors are encoder-predicted minus GPS distance. All-eight fit errors are in-sample.\n\n## Whole-section validation\n\n'
    for s,f in scales.items():
        v=f['first_four_fit_last_four_validation'];l=f['leave_one_section_out']
        report+=f"- {s}: fit first four at {f['first_four_fit_ticks_per_metre']:.3f} ticks/m, then predict the later four: RMSE {v['rmse_m']:.3f} m, maximum absolute error {v['max_abs_error_m']:.3f} m. Fitting the later four alone gives {f['last_four_fit_ticks_per_metre']:.3f} ticks/m.\n"
        report+=f"- {s}: leave each whole section out, fit the other seven, predict the omitted section: RMSE {l['rmse_m']:.3f} m, maximum absolute error {l['max_abs_error_m']:.3f} m. Refit scale range {f['leave_one_section_out_scale_range'][0]:.3f}–{f['leave_one_section_out_scale_range'][1]:.3f} ticks/m.\n"
    report+='\nThe final candidate uses all eight. The first-four/later-four test checks chronological separation; leave-one-out checks section influence but its training sets overlap. These are same-run forward sections, not independent runs or reverse validation. No population confidence interval or short-outage pose accuracy is established by this fit.\n\n'
    report+='Recorded heading/intermediate-position flags are preserved in fit_results.json rather than used to discard the selected sections. All start/end position brackets were previously checked as RTK Fixed, including section 8; interior loss does not automatically invalidate endpoint displacement. Exact receiver epochs remain unavailable, but this fit uses the agreed host-time alignment. Wheel boundary fractions, source counts, position interpolation and original flags are preserved in the input extraction.\n\n'
    report+='This package is separate from the earlier strict-window fit. No services, firmware, raw transport, reviewed-calibration flag or motion authorization are changed. Next use is offline wheel/IMU pose prediction with these empirical constants.\n\n'
    report+='Reproduce from the repository root (the package retains the exact extraction input as selected_sections.json):\n\n```powershell\npython field_testing/tools/fit_user_encoder_sections_20261120.py --input field_testing/sites/62_Collins_multi_boundary_20260915/analysis/20261120_user_selected_encoder_fit/selected_sections.json --output-dir field_testing/sites/62_Collins_multi_boundary_20260915/analysis/20261120_user_selected_encoder_fit\n```\n'
    (out/'REPORT.md').write_text(report,encoding='utf-8')
    provenance=dict(input=str(source.resolve()),input_sha256=sha(source),analyzer_sha256=sha(Path(__file__)),
                    verified_original_sources=data['source_hashes'])
    (out/'source_sha256.json').write_text(json.dumps(provenance,indent=2)+'\n',encoding='utf-8')
    (out/'SHA256SUMS').write_text(''.join(sha(p)+'  '+p.name+'\n' for p in sorted(out.iterdir()) if p.is_file() and p.name!='SHA256SUMS'))
    print(json.dumps(dict(total_reference_distance_m=total_distance,scales=scales),indent=2))

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True);parser.add_argument('--output-dir',type=Path,required=True)
    args=parser.parse_args();build(args.input.resolve(),args.output_dir.resolve())
