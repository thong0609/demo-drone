"""Run reproducible planner-only snapshots: python3 -m drone_bench.cli.plan_swept_volume."""
import argparse
import copy
import hashlib
import json
import math
import time
from pathlib import Path
from ..environment.swept import SweptEnvironment
from ..algorithms.planners.paper_tube import build_paper_tube
from ..algorithms.planners.swept_volume import update_tube
from ..environment.world import Environment
from ..io.benchmark import write_json
from ..visualization.swept_replay import write_replay


def geometry(tube):
    value = dict(points=tube.points, radii=tube.radii)
    if hasattr(tube, 'curve'):
        value['curve'] = tube.curve
    return value


def run(config, output):
    out = Path(output)
    out.mkdir(parents=True, exist_ok=True)
    resolved = copy.deepcopy(config)
    write_json(out / 'resolved_config.json', resolved)
    sources = ['drone_bench/environment/swept.py',
               'drone_bench/algorithms/planners/swept_volume.py',
               'drone_bench/algorithms/planners/tube_quality.py',
               'drone_bench/algorithms/planners/paper_tube.py',
               'drone_bench/algorithms/planners/enhanced_rrt.py',
               'drone_bench/algorithms/planners/minimum_snap.py',
               'drone_bench/tubes/model.py', 'drone_bench/tubes/polynomial.py',
               'drone_bench/cli/plan_swept_volume.py',
               'drone_bench/visualization/swept_replay.py',
               'drone_bench/visualization/swept_replay.html',
               'drone_bench/environment/world.py', 'drone_bench/environment/obstacles.py',
               'drone_bench/common/linear.py', 'drone_bench/common/vectors.py']
    root = Path(__file__).resolve().parents[2]
    write_json(out / 'source_hashes.json', {
        name: hashlib.sha256((root / name).read_bytes()).hexdigest() for name in sources})
    world, planner = config['world'], config['planner']
    times = config['update_times']
    if (not times or any(not math.isfinite(t) for t in times)
            or any(b <= a for a, b in zip(times, times[1:]))):
        raise ValueError('update_times must be finite and strictly increasing')
    initial_started = time.perf_counter()
    try:
        previous, initial_report = build_paper_tube(Environment(world), planner, config['seed'])
        initial_data = dict(tube=geometry(previous), planning=initial_report, error=None)
    except (ValueError, RuntimeError, ArithmeticError) as error:
        previous = None
        initial_data = dict(tube=None, planning=None, error=str(error))
    initial_seconds = time.perf_counter() - initial_started
    write_json(out / 'initial_tube.json', initial_data)
    records = []
    replay_updates = []
    for index, now in enumerate(times):
        # Synthetic measurement generator. Only current center/velocity enter the
        # predictor; it is not given the future simulator trajectory.
        observations = []
        for obstacle in config['obstacles']:
            item = copy.deepcopy(obstacle)
            if now < item['timestamp']:
                raise ValueError('Update precedes synthetic initial state')
            item['center'] = [p + v * (now - item['timestamp'])
                              for p, v in zip(item['center'], item['velocity'])]
            item['timestamp'] = now
            observations.append(item)
        env = SweptEnvironment(world, observations, now, config['horizon'],
                               config['prediction_margin'], max_age=0.)
        seed=config['seed'] if planner.get('seed_policy')=='fixed' else config['seed']+index
        tube, report = update_tube(env, planner, seed, previous,
                                   config['deadline_seconds'])
        report.update(update_index=index, observations=observations)
        # Invalidate old plans on failure; a later snapshot may attempt recovery.
        previous = tube
        update = dict(report=report, tube=geometry(tube) if tube else None)
        write_json(out / f'update_{index:03d}.json', update)
        records.append(report)
        replay_updates.append(update)
    latencies = sorted(r['elapsed_seconds'] for r in records)
    def percentile(p):
        return latencies[max(0, math.ceil(p * len(latencies)) - 1)]
    summary = dict(scope='planner-only; no controller or swarm execution',
                   config_sha256=hashlib.sha256(json.dumps(resolved, sort_keys=True).encode()).hexdigest(),
                   updates=len(records), accepted=sum(r['accepted'] for r in records),
                   initial_static_planning_seconds=initial_seconds,
                   initial_static_planning_error=initial_data['error'],
                   replanned=sum(r['status'] == 'replanned' for r in records),
                   retained=sum(r['status'] == 'retained' for r in records),
                   failures=sum(not r['accepted'] for r in records),
                   deadline_misses=sum(r['deadline_missed'] for r in records),
                   mean_seconds=sum(latencies)/len(latencies), p95_seconds=percentile(.95),
                   p99_seconds=percentile(.99), max_seconds=max(latencies),
                   timing_scope='update_tube including checks; excludes snapshot creation, initial static planning and export',
                   realtime_demonstrated=False)
    write_json(out / 'summary.json', summary)
    write_replay(out / 'report.html', resolved, replay_updates, initial_data['tube'], summary)
    return summary


def run_sweep(config, output):
    """One-factor scenarios, including declared failure cases; not success filtering."""
    cases = []
    def variant(name, change):
        item = copy.deepcopy(config)
        change(item)
        cases.append((name, item))
    variant('stationary', lambda c: [o.update(velocity=[0, 0, 0]) for o in c['obstacles']])
    variant('crossing_default', lambda c: None)
    for radius in (3, 18):
        variant(f'radius_{radius}', lambda c, r=radius: c['obstacles'][0].update(radius=r))
    for speed in (5, 45):
        variant(f'speed_{speed}', lambda c, v=speed: c['obstacles'][0].update(velocity=[0, -v, 0]))
    for horizon in (1, 6):
        variant(f'horizon_{horizon}', lambda c, h=horizon: c.update(horizon=h))
    variant('one_obstacle', lambda c: c.update(obstacles=c['obstacles'][:1]))
    variant('no_obstacles', lambda c: c.update(obstacles=[]))
    def blocked(c):
        c['obstacles'] = [dict(id='anchor_blocker', shape='sphere', center=c['world']['start'],
                               radius=4, velocity=[0, 0, 0], timestamp=0)]
    variant('start_blocked_expected', blocked)
    def future_start(c):
        start = c['world']['start']
        c['obstacles'] = [dict(id='future_anchor', shape='sphere',
                               center=[start[0], start[1]+40, start[2]],
                               radius=4, velocity=[0, -20, 0], timestamp=0)]
    variant('future_start_conservative_failure', future_start)
    results = []
    for name, item in cases:
        results.append(dict(case=name, summary=run(item, Path(output) / name)))
    write_json(Path(output) / 'sweep_summary.json', results)
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', help='Defaults to replay demo; --sweep defaults to original fixture')
    parser.add_argument('--output', default='artifacts/swept_volume')
    parser.add_argument('--sweep', action='store_true', help='Run size/speed/horizon and failure fixtures')
    args = parser.parse_args()
    config_path = args.config or ('configs/experiments/swept_volume.json' if args.sweep
                                  else 'configs/experiments/swept_volume_replay.json')
    config = json.loads(Path(config_path).read_text())
    if args.sweep:
        results = run_sweep(config, args.output)
        print(json.dumps([dict(case=r['case'], accepted=r['summary']['accepted'],
                               failures=r['summary']['failures']) for r in results], indent=2))
        # This is a diagnostic suite with deliberate failures, not a pass gate.
        return 0
    summary = run(config, args.output)
    print(json.dumps(summary, indent=2))
    return 2 if summary['failures'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
