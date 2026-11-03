"""Paired vector-field/DMPC experiment on byte-identical frozen initial states."""
import argparse
import copy
import csv
import hashlib
import json
from pathlib import Path
from ..io.benchmark import load_scenario,load_benchmark,write_json
from ..algorithms.controllers.paper_outer import PaperOuter
from ..algorithms.controllers.paper_inner import PaperInner
from ..algorithms.controllers.dmpc.controller import DMPC
from ..algorithms.controllers.dmpc.calibration import calibrate
from ..simulation.runner import run
from ..visualization.results import export_results


def fingerprint(value):return hashlib.sha256(json.dumps(value,sort_keys=True).encode()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',default='configs/experiments/paper_vb_dmpc.json')
    parser.add_argument('--output',default='artifacts/outer_comparison')
    parser.add_argument('--duration',type=float)
    parser.add_argument('--validation',help='Require a passed, matching short-validation gates.json')
    parser.add_argument('--only',choices=['both','vector_field','dmpc'],default='both')
    args=parser.parse_args()
    settings=load_scenario(args.config);cfg=load_scenario(settings['base_config'])
    if args.validation:
        from .validate_dmpc import validation_fingerprint
        gate=load_scenario(args.validation)
        if not gate['passed'] or gate['fingerprint']!=validation_fingerprint(settings):
            parser.error('Short validation missing, failed, or stale; run validate_dmpc first')
    if args.duration is not None:cfg['simulation']['duration']=args.duration
    if cfg['simulation']['duration']<=0:parser.error('Duration must be positive')
    out=Path(args.output);inputs=out/'inputs';inputs.mkdir(parents=True,exist_ok=True)
    payload=load_scenario(settings['benchmark']);env,tube=load_benchmark(settings['benchmark'])
    initial=load_scenario(settings['initial_state'])['states']
    hashes=dict(benchmark=fingerprint(payload),initial_state=fingerprint(initial),inner=fingerprint(cfg['inner']),
                dynamics=fingerprint(cfg['dynamics']),simulation=fingerprint(cfg['simulation']))
    write_json(inputs/'benchmark.json',payload);write_json(inputs/'initial_state.json',{'states':initial})
    write_json(inputs/'base_config.json',cfg);write_json(inputs/'dmpc_config.json',settings)
    write_json(inputs/'manifest.json',hashes)
    write_json(inputs/'predictor_calibration.json',calibrate(cfg))
    source_paths=[Path('drone_bench/algorithms/controllers/paper_inner.py'),Path('drone_bench/simulation/dynamics.py')]
    source_paths+=list(Path('drone_bench/algorithms/controllers/dmpc').glob('*.py'))
    source_paths+=list(Path('drone_bench/algorithms/controllers/dmpc').glob('*.c'))
    source_hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in source_paths}
    write_json(inputs/'source_hashes.json',source_hashes)
    results={}
    names=['vector_field','dmpc'] if args.only=='both' else [args.only]
    for name in names:
        target=out/name;target.mkdir(exist_ok=True)
        current=copy.deepcopy(cfg)
        current['controller_name']='Enhanced vector field' if name=='vector_field' else 'DMPC (static tube, velocity reference)'
        if name=='dmpc':current['dmpc']=settings['dmpc']
        outer=PaperOuter(tube,cfg['outer']) if name=='vector_field' else DMPC(tube,cfg['outer'],cfg['inner'],settings['dmpc'])
        print(f'Running {name}',flush=True)
        frames,history,summary=run(env,tube,current,target,outer,PaperInner(cfg['inner'],cfg['dynamics']),initial_states=initial)
        write_json(target/'resolved_config.json',current);write_json(target/'manifest.json',hashes)
        write_json(target/'benchmark.json',payload)
        write_json(target/'source_hashes.json',source_hashes)
        if name=='dmpc':
            write_json(target/'solver_summary.json',outer.stats);write_json(target/'solver_cycles.json',outer.history)
            write_json(target/'prediction_errors.json',outer.prediction_records)
            with (target/'solver.jsonl').open('w') as f:
                for record in outer.records:f.write(json.dumps(record)+'\n')
            summary['solver']=outer.stats;write_json(target/'summary.json',summary)
        if frames:export_results(payload,tube,frames,history,summary,current,target)
        results[name]=summary
    # A vector-only run is also a regression check against the original stored run.
    reference=Path(settings['initial_state']).with_name('summary.json')
    if 'vector_field' in results and args.duration is None and reference.exists():
        old=load_scenario(reference);new=results['vector_field']
        keys=['finish_time','min_pair_distance','max_pair_distance','min_boundary_clearance','max_speed','max_input']
        regression={k:abs(new[k]-old[k]) for k in keys if new[k] is not None and old[k] is not None}
        write_json(out/'baseline_regression.json',dict(differences=regression,passed=all(v<1e-8 for v in regression.values())))
    for name in ('vector_field','dmpc'):
        if name not in results and (out/name/'summary.json').exists():
            manifest=load_scenario(out/name/'manifest.json')
            if manifest!=hashes:raise ValueError('Cannot combine results with different frozen inputs')
            results[name]=load_scenario(out/name/'summary.json')
    write_json(out/'comparison.json',dict(shared_inputs=hashes,results=results))
    from ..visualization.comparison import export_comparison
    export_comparison(out,results,hashes)
    print(json.dumps({name:{k:s[k] for k in ['completed','finish_time','max_speed','min_pair_distance','min_boundary_clearance','error','success']} for name,s in results.items()},indent=2))

if __name__=='__main__':main()
