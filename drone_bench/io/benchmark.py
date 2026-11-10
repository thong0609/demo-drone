"""Scenario loading and portable benchmark serialization."""
import hashlib
import json
from pathlib import Path
from ..environment.world import Environment
from ..tubes.model import Tube


def load_scenario(path):
    return json.loads(Path(path).read_text())


def make_payload(config, tube):
    fingerprint = hashlib.sha256(json.dumps(config, sort_keys=True).encode()).hexdigest()
    report = tube.validate(Environment(config), config['tube']['obstacle_margin'])
    report.update(scenario_id=config['id'], scenario_sha256=fingerprint,
                  samples=len(tube.points), safety_radius=config['safety_radius'])
    geometry=dict(points=tube.points,radii=tube.radii,
                  definition=getattr(tube,'representation','piecewise-linear centerline and radius; normal disks'))
    if hasattr(tube,'curve'): geometry['curve']=tube.curve
    return dict(schema_version=1, units='metres', scenario=config,
                scenario_sha256=fingerprint, report=report,
                tube=geometry)


def write_json(path, data):
    Path(path).write_text(json.dumps(data, indent=2) + '\n')


def load_benchmark(path):
    """Load the same frozen world/tube for every controller."""
    payload = load_scenario(path)
    if payload['schema_version'] != 1 or payload['units'] != 'metres':
        raise ValueError('Unsupported benchmark schema or units')
    config = payload['scenario']
    actual = hashlib.sha256(json.dumps(config, sort_keys=True).encode()).hexdigest()
    if actual != payload['scenario_sha256']:
        raise ValueError('Scenario fingerprint mismatch')
    env = Environment(config)
    geometry=payload['tube']
    if 'curve' in geometry:
        from ..tubes.polynomial import PolynomialTube
        tube=PolynomialTube(geometry['points'],geometry['radii'],geometry['curve'])
    else:
        tube=Tube(geometry['points'],geometry['radii'])
    tube.validate(env, config['tube']['obstacle_margin'])
    return env, tube
