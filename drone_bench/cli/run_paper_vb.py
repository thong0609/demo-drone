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
    p.add_argument('--output',default='artifacts/paper_vb')
    p.add_argument('--benchmark',help='Reuse a frozen benchmark instead of replanning')
    p.add_argument('--dt',type=float);p.add_argument('--duration',type=float)
    p.add_argument('--plan-only',action='store_true')
    args=p.parse_args();cfg=load_scenario(args.config)
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
    else:
        env=Environment(load_scenario(cfg['scenario']))
        tube,planning=build_paper_tube(env,cfg['planner'],cfg['seed'])
    payload=make_payload(env.config,tube)
    serialized=json.dumps(payload,sort_keys=True).encode()
    write_json(out/'benchmark.json',payload);write_json(out/'planning.json',planning)
    write_json(out/'resolved_config.json',cfg)
    write_json(out/'manifest.json',{'benchmark_sha256':hashlib.sha256(serialized).hexdigest(),'seed':cfg['seed'],'config':cfg,
                                  'provenance':cfg['provenance'],'planner':planning.get('algorithm','frozen benchmark')})
    write_viewer(payload,tube,out/'environment.html')
    if args.plan_only:
        print(json.dumps(payload['report'],indent=2));return
    frames,history,summary=run(env,tube,cfg,out,PaperOuter(tube,cfg['outer']),PaperInner(cfg['inner'],cfg['dynamics']))
    from ..visualization.results import export_results
    export_results(payload,tube,frames,history,summary,cfg,out)
    print(json.dumps(summary,indent=2))
    if not summary['success']: raise SystemExit(2)

if __name__=='__main__':main()
