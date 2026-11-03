"""Short validation gates. Run these before the complete paired experiment."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
from ..io.benchmark import load_scenario, load_benchmark, write_json
from ..algorithms.controllers.dmpc.controller import DMPC
from ..algorithms.controllers.paper_inner import PaperInner
from ..simulation.runner import run


def validation_fingerprint(settings):
    digest=hashlib.sha256(json.dumps(settings,sort_keys=True).encode())
    for field in ('base_config','benchmark','initial_state'):
        digest.update(Path(settings[field]).read_bytes())
    sources=list(Path('drone_bench/algorithms/controllers/dmpc').glob('*.py'))
    sources+=list(Path('drone_bench/algorithms/controllers/dmpc').glob('*.c'))
    sources+=[Path('drone_bench/algorithms/controllers/paper_inner.py'),Path('drone_bench/simulation/dynamics.py'),Path('drone_bench/simulation/runner.py')]
    for path in sorted(sources):digest.update(path.read_bytes())
    return digest.hexdigest()


def validate(config='configs/experiments/paper_vb_dmpc.json',output='artifacts/dmpc_validation'):
    settings=load_scenario(config);base=load_scenario(settings['base_config'])
    env,tube=load_benchmark(settings['benchmark']);initial=load_scenario(settings['initial_state'])['states']
    out=Path(output);out.mkdir(parents=True,exist_ok=True)
    reports=[]
    for count,duration in [(1,3),(2,3),(4,3),(27,2)]:
        cfg=copy.deepcopy(base);cfg['simulation']['duration']=duration
        print(f'Short validation: {count} UAVs / {duration} seconds',flush=True)
        ctrl=DMPC(tube,cfg['outer'],cfg['inner'],settings['dmpc'])
        _,_,summary=run(env,tube,cfg,out/f'uav_{count}',ctrl,PaperInner(cfg['inner'],cfg['dynamics']),initial_states=initial[:count])
        passed=summary['error'] is None and summary['constraints_passed']
        reports.append(dict(agents=count,duration=duration,passed=passed,summary=summary))
        if not passed:break
    data=dict(fingerprint=validation_fingerprint(settings),passed=len(reports)==4 and all(r['passed'] for r in reports),
              scope='Short smoke/integration gates; not a proof of feasibility for the full mission.',checks=reports)
    write_json(out/'gates.json',data)
    print(json.dumps({k:data[k] for k in ('passed','fingerprint')},indent=2))
    return data


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',default='configs/experiments/paper_vb_dmpc.json')
    parser.add_argument('--output',default='artifacts/dmpc_validation')
    args=parser.parse_args()
    if not validate(args.config,args.output)['passed']:raise SystemExit(2)

if __name__=='__main__':main()
