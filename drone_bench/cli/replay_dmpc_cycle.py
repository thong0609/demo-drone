"""Reproduce one failed outer update from the runner's exact causal checkpoint."""
import argparse
import json
from pathlib import Path
from ..io.benchmark import load_scenario,load_benchmark,write_json
from ..algorithms.controllers.dmpc.controller import DMPC


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('checkpoint')
    parser.add_argument('--config',default='configs/experiments/paper_vb_dmpc.json')
    parser.add_argument('--output',default='/tmp/dmpc_replayed_cycle.json')
    args=parser.parse_args()
    checkpoint=load_scenario(args.checkpoint);settings=load_scenario(args.config)
    base=load_scenario(settings['base_config']);_,tube=load_benchmark(settings['benchmark'])
    ctrl=DMPC(tube,base['outer'],base['inner'],settings['dmpc']);ctrl.import_state(checkpoint['outer'])
    states=checkpoint['states'];time=checkpoint['time'];active=checkpoint['active']
    try:
        commands,_=ctrl.commands([s[:3] for s in states],active,time=time,states=states)
        result=dict(passed=True,time=time,commands=commands,solver=ctrl.stats)
    except (ValueError,OverflowError) as error:
        result=dict(passed=False,time=time,error=str(error),solver=ctrl.stats)
    write_json(args.output,result)
    print(json.dumps(result,indent=2))
    if not result['passed']:raise SystemExit(2)

if __name__=='__main__':main()
