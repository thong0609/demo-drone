"""Build shared geometry and viewer: python3 -m drone_bench.cli.build_environment."""
import argparse
import json
from pathlib import Path
from ..environment.world import Environment
from ..algorithms.planners.reference import build_reference_tube
from ..io.benchmark import load_scenario, make_payload, write_json
from ..visualization.export import write_viewer


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', default='configs/scenarios/benchmark_v1.json')
    parser.add_argument('--output', default='artifacts')
    args = parser.parse_args()
    config = load_scenario(args.scenario)
    tube = build_reference_tube(Environment(config))
    payload = make_payload(config, tube)
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    write_json(out / 'benchmark.json', payload)
    write_json(out / 'validation.json', payload['report'])
    write_viewer(payload, tube, out / 'viewer.html')
    print(json.dumps(payload['report'], indent=2))
    print(f'Open {out / "viewer.html"} in a browser.')


if __name__ == '__main__':
    main()
