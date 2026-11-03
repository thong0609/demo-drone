import json
from pathlib import Path
import tempfile
import unittest
from drone_bench.io.benchmark import load_scenario, make_payload, write_json, load_benchmark
from drone_bench.environment.world import Environment
from drone_bench.algorithms.planners.reference import build_reference_tube


class BenchmarkIOTests(unittest.TestCase):
    def test_export_load_preserves_geometry_and_rejects_changed_world(self):
        config = load_scenario('configs/scenarios/benchmark_v1.json')
        tube = build_reference_tube(Environment(config))
        payload = make_payload(config, tube)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'benchmark.json'
            write_json(path, payload)
            env, restored = load_benchmark(path)
            self.assertEqual(restored.points, tube.points)
            self.assertEqual(restored.radii, tube.radii)
            self.assertEqual(env.config, config)
            modified = json.loads(path.read_text())
            modified['scenario']['goal'][0] += 1
            write_json(path, modified)
            with self.assertRaisesRegex(ValueError, 'fingerprint'):
                load_benchmark(path)
