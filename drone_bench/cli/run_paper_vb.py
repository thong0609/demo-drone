"""Run paper Section V-B on the shared custom world."""
import argparse
import hashlib
import json
from pathlib import Path
from ..environment.world import Environment
from ..algorithms.planners.paper_tube import build_paper_tube
from ..io.benchmark import load_scenario,load_benchmark,make_payload,write_json
from ..visualization.export import write_viewer
from ..simulation.runner import run
from ..algorithms.controllers.paper_outer import PaperOuter
from ..algorithms.controllers.paper_inner import PaperInner


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',default='configs/experiments/paper_vb.json')
    p.add_argument('--output',default='artifacts/paper_vb_continuous')
    p.add_argument('--benchmark',help='Reuse a frozen benchmark instead of replanning')
    p.add_argument('--restore-polynomial',action='store_true',help='Restore continuous coefficients from the benchmark sibling planning.json, checking the original samples')
    p.add_argument('--initial-state',help='Reuse frozen positions/velocities; initialize filter and adaptive states for the current command per Eq. 41')
    p.add_argument('--dt',type=float);p.add_argument('--duration',type=float)
    p.add_argument('--plan-only',action='store_true')
    args=p.parse_args();cfg=load_scenario(args.config)
    if args.restore_polynomial and not args.benchmark:p.error('--restore-polynomial requires --benchmark')
    if args.dt is not None: cfg['simulation']['dt']=args.dt
    if args.duration is not None: cfg['simulation']['duration']=args.duration
    if cfg['simulation']['dt']<=0 or cfg['simulation']['duration']<=0: p.error('dt and duration must be positive')
    out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    if args.benchmark:
        env,tube=load_benchmark(args.benchmark)
        planning={'source':str(Path(args.benchmark).resolve()),'reused':True}
        source_planning=Path(args.benchmark).with_name('planning.json')
        if source_planning.exists():
            original=load_scenario(source_planning)
            if 'waypoints' in original: planning.update(original)
        if args.restore_polynomial:
            if hasattr(tube,'curve'):p.error('Benchmark already contains polynomial coefficients')
            from ..tubes.polynomial import PolynomialTube
            if not all(k in planning for k in ('centerline_coefficients','radius_coefficients','durations','waypoints')):
                p.error('Restoring a polynomial requires complete original planning.json')
            import math
            from ..common.vectors import norm,sub
            parameters=[]
            for s,(a,b) in enumerate(zip(planning['waypoints'],planning['waypoints'][1:])):
                n=max(30,math.ceil(norm(sub(b,a))/cfg['planner']['sample_spacing']))
                parameters.extend([s,j/n] for j in range(n))
            parameters.append([len(planning['durations'])-1,1])
            if len(parameters)!=len(tube.points):p.error('Original polynomial sampling does not match benchmark')
            curve={k:planning[k] for k in ('centerline_coefficients','radius_coefficients','durations')}
            curve['sample_parameters']=parameters
            restored=PolynomialTube(tube.points,tube.radii,curve)
            for (s,t),point,radius in zip(parameters,tube.points,tube.radii):
                center,r,_,_=restored.sample(s,t)
                if norm(sub(center,point))>1e-5 or abs(r-radius)>1e-5:p.error('Polynomial coefficients do not reproduce frozen benchmark')
            tube=restored;planning['restored_polynomial']=True
    else:
        env=Environment(load_scenario(cfg['scenario']))
        tube,planning=build_paper_tube(env,cfg['planner'],cfg['seed'])
    payload=make_payload(env.config,tube)
    serialized=json.dumps(payload,sort_keys=True).encode()
    write_json(out/'benchmark.json',payload);write_json(out/'planning.json',planning)
    write_json(out/'resolved_config.json',cfg)
    write_json(out/'source_hashes.json',{str(path):hashlib.sha256(path.read_bytes()).hexdigest()
                                       for path in sorted(Path('drone_bench').rglob('*.py'))})
    write_json(out/'manifest.json',{'benchmark_sha256':hashlib.sha256(serialized).hexdigest(),'seed':cfg['seed'],'config':cfg,
                                  'provenance':cfg['provenance'],'planner':planning.get('algorithm','frozen benchmark')})
    write_viewer(payload,tube,out/'environment.html')
    if args.plan_only:
        print(json.dumps(payload['report'],indent=2));return
    inner=PaperInner(cfg['inner'],cfg['dynamics'])
    write_json(out/'inner_certificate.json',inner.certificate)
    initial=None
    if args.initial_state:
        frozen=load_scenario(args.initial_state)['states']
        initial_commands,_=PaperOuter(tube,cfg['outer']).commands([s[:3] for s in frozen],list(range(len(frozen))))
        initial=[inner.initial_state(s[:3],s[3:6],initial_commands[i]) for i,s in enumerate(frozen)]
        write_json(out/'initialization.json',dict(source=str(Path(args.initial_state).resolve()),
                   reused='positions and physical velocities',
                   adaptive_initialization='Eq. 41 zeta=vc; e_hat=e; xi=-C*e; zero weights, auxiliary and robust gains'))
    frames,history,summary=run(env,tube,cfg,out,PaperOuter(tube,cfg['outer']),inner,initial_states=initial)
    from ..visualization.results import export_results
    export_results(payload,tube,frames,history,summary,cfg,out)
    print(json.dumps(summary,indent=2))
    if not summary['success']: raise SystemExit(2)

if __name__=='__main__':main()
