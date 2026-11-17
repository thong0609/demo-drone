"""A2: matched horizon sweep, complete replay report, configurable horizon choices."""
import argparse
import copy
import csv
import hashlib
import html
import json
import math
import platform
import statistics
from pathlib import Path
from .evaluate_swept_volume import cases
from .plan_swept_volume import run
from ..algorithms.planners.swept_volume import check_tube
from ..algorithms.planners.tube_quality import shape_metrics, displacement
from ..environment.swept import SweptEnvironment
from ..tubes.model import Tube
from ..tubes.polynomial import PolynomialTube
from ..io.benchmark import write_json


def load_tube(g):
    if not g:return None
    return PolynomialTube(g['points'],g['radii'],g['curve']) if 'curve' in g else Tube(g['points'],g['radii'])


def validate_study(study):
    values=study['horizons']
    if not values or any(not isinstance(h,(int,float)) or not math.isfinite(h) or h<=0 for h in values):
        raise ValueError('Horizons must be finite, positive numbers')
    if len(set(values))!=len(values):raise ValueError('Duplicate horizons')
    if not study['seeds'] or len(set(study['seeds']))!=len(study['seeds']):raise ValueError('Seeds must be nonempty and unique')
    for key in ('reference_horizon','last_update_interval'):
        if not math.isfinite(study[key]) or study[key]<=0:raise ValueError('Invalid '+key)


def horizon_configs(base,study):
    validate_study(study)
    # Drop old horizon_1/6 cases: H itself is the independent variable now.
    matrix=[(n,c) for n,c in cases(base) if not n.startswith('horizon_')]
    selected=study.get('case_names')
    if selected is not None:
        unknown=set(selected)-{n for n,_ in matrix}
        if unknown:raise ValueError('Unknown cases: '+str(sorted(unknown)))
        matrix=[(n,c) for n,c in matrix if n in selected]
    if not matrix:raise ValueError('No cases selected')
    result=[]
    for name,c in matrix:
        for seed in study['seeds']:
            for horizon in sorted(study['horizons']):
                item=copy.deepcopy(c);item['seed']=seed;item['horizon']=horizon
                result.append((name,seed,horizon,item))
    return result


def htag(h):return format(h,'g').replace('.','p')


def evaluate(base,study,output):
    variants=horizon_configs(base,study);out=Path(output);out.mkdir(parents=True,exist_ok=True)
    write_json(out/'study_config.json',study)
    write_json(out/'base_config.json',base)
    root=Path(__file__).resolve().parents[2]
    sources=list((root/'drone_bench').rglob('*.py'))+[root/'drone_bench/visualization/swept_replay.html',root/'drone_bench/visualization/horizon_report.html']
    write_json(out/'source_hashes.json',{str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
    records=[];runs=[]
    for index,(name,seed,horizon,cfg) in enumerate(variants):
        relative=Path('runs')/name/f'seed_{seed}'/('H_'+htag(horizon))
        target=out/relative
        summary=run(cfg,target)
        initial=load_tube(json.loads((target/'initial_tube.json').read_text())['tube'])
        previous=initial
        for j,now in enumerate(cfg['update_times']):
            raw=json.loads((target/f'update_{j:03d}.json').read_text());report=raw['report'];tube=load_tube(raw['tube'])
            interval=cfg['update_times'][j+1]-now if j+1<len(cfg['update_times']) else study['last_update_interval']
            checks={}
            for kind,h in [('reference',study['reference_horizon']),('next_update',interval)]:
                valid=False;reason='No accepted tube';certificate=None
                if tube:
                    env=SweptEnvironment(cfg['world'],report['observations'],now,h,cfg['prediction_margin'])
                    try:
                        certificate=check_tube(tube,env,cfg['world']['tube']['obstacle_margin'],cfg['planner']['radius_floor'])
                        valid=True;reason=None
                    except ValueError as error:reason=str(error)
                checks[kind]=dict(horizon=h,certified=valid,reason=reason,certificate=certificate)
            path=(report.get('planning') or {}).get('waypoints')
            if tube and path is None:
                path=[tube.sample(s,0)[0] for s in range(tube.spans)]+[tube.sample(tube.spans-1,1)[0]]
            quality=shape_metrics(tube,path) if tube else {}
            change=displacement(previous,tube) if previous and tube else {}
            previous=tube
            record=dict(case=name,seed=seed,horizon=horizon,time=now,status=report['status'],accepted=report['accepted'],
                        reference_certified=checks['reference']['certified'],next_update_certified=checks['next_update']['certified'],
                        horizon_covers_update_interval=horizon>=interval,update_interval=interval,
                        elapsed_seconds=report['elapsed_seconds'],deadline_missed=report['deadline_missed'],
                        length=quality.get('length'),min_radius=quality.get('min_radius'),curvature_sampled=quality.get('sampled_max_curvature'),
                        change_mean=change.get('mean'),error=report.get('error'),
                        reference_reason=checks['reference']['reason'],next_update_reason=checks['next_update']['reason'],
                        replay=(relative/'report.html').as_posix())
            records.append(record)
            write_json(target/f'evaluation_{j:03d}.json',dict(record=record,checks=checks,quality=quality,change=change))
        runs.append(dict(case=name,label=cfg.get('case_label',name),seed=seed,horizon=horizon,
                         replay=(relative/'report.html').as_posix(),summary=summary))
        if (index+1)%len(study['horizons'])==0:
            print(f'Finished {name}, seed {seed} ({index+1}/{len(variants)} runs)',flush=True)
            write_json(out/'records.json',records)
    write_json(out/'runs.json',runs)
    with (out/'metrics.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(records[0]));writer.writeheader();writer.writerows(records)
    summaries=[]
    for horizon in sorted(study['horizons']):
        group=[r for r in records if r['horizon']==horizon];times=sorted(r['elapsed_seconds'] for r in group)
        def mean(key):
            values=[r[key] for r in group if r['accepted'] and r[key] is not None]
            return statistics.mean(values) if values else None
        summaries.append(dict(horizon=horizon,updates=len(group),accepted=sum(r['accepted'] for r in group),
                              reference_certified=sum(r['reference_certified'] for r in group),
                              next_update_certified=sum(r['next_update_certified'] for r in group),
                              uncovered_update_intervals=sum(not r['horizon_covers_update_interval'] for r in group),
                              mean_seconds=statistics.mean(times),p95_seconds=times[math.ceil(.95*len(times))-1],
                              p99_seconds=times[math.ceil(.99*len(times))-1],max_seconds=max(times),
                              deadline_misses=sum(r['deadline_missed'] for r in group),mean_length_accepted=mean('length'),
                              mean_change_accepted=mean('change_mean'),statuses={s:sum(r['status']==s for r in group) for s in sorted({r['status'] for r in group})}))
    write_json(out/'summary.json',summaries)
    # Compare lengths only where BOTH horizon choices have an accepted candidate.
    reference=min(study['horizons'],key=lambda h:abs(h-study['reference_horizon']))
    paired=[]
    for horizon in sorted(study['horizons']):
        deltas=[]
        for r in records:
            if r['horizon']!=horizon or not r['accepted']:continue
            a=next(a for a in records if a['horizon']==reference and all(a[k]==r[k] for k in ('case','seed','time')))
            if a['accepted']:deltas.append(r['length']-a['length'])
        paired.append(dict(horizon=horizon,baseline_horizon=reference,matched_accepted=len(deltas),mean_length_delta=statistics.mean(deltas) if deltas else None))
    write_json(out/'paired_comparison.json',paired)
    write_json(out/'protocol.json',dict(study=study,variants=len(variants),updates=len(records),
                                      varied='Only horizon within each case/seed; planner/observations/seed/update times/margins unchanged',
                                      platform=platform.platform(),python=platform.python_version(),
                                      timing='Planner update only, excludes initial static plan, snapshot creation, evaluation and export',
                                      reference_check='Extra conservative geometry diagnostic at fixed horizon, not physical collision verdict',
                                      intervals='Last snapshot uses explicitly configured last_update_interval; no UAV execution',
                                      independent_trials=False,held_out=False))
    write_report(out,study,runs,records,summaries)
    return summaries


def write_report(out,study,runs,records,summaries):
    data=json.dumps(dict(study=study,runs=runs,records=records,summaries=summaries),ensure_ascii=False).replace('<','\\u003c')
    template=Path(__file__).resolve().parents[1]/'visualization/horizon_report.html'
    (out/'report.html').write_text(template.read_text().replace('__HORIZON_DATA__',data))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--study',default='configs/experiments/swept_horizon_study.json')
    parser.add_argument('--horizons',nargs='+',type=float,help='Override tested horizon values, seconds')
    parser.add_argument('--output',default='artifacts/swept_horizon_study')
    args=parser.parse_args();study=json.loads(Path(args.study).read_text())
    if args.horizons is not None:study['horizons']=args.horizons
    base=json.loads(Path(study['base_config']).read_text())
    print(json.dumps(evaluate(base,study,args.output),indent=2))


if __name__=='__main__':main()
