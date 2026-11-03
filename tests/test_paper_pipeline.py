import copy
import tempfile
import unittest
from pathlib import Path
from drone_bench.io.benchmark import load_scenario,load_benchmark
from drone_bench.environment.world import Environment
from drone_bench.algorithms.planners.paper_tube import build_paper_tube
from drone_bench.algorithms.controllers.paper_outer import PaperOuter
from drone_bench.algorithms.controllers.paper_inner import PaperInner
from drone_bench.simulation.runner import run


class PaperPipelineTests(unittest.TestCase):
    def test_planning_pipeline_clear_world_and_determinism(self):
        cfg=load_scenario('configs/experiments/paper_vb.json')['planner']
        cfg.update(iterations=12,goal_bias=1,step=60,goal_radius=100,clearance=20)
        world=load_scenario('configs/scenarios/benchmark_v1.json')
        world.update(obstacles=[],start=[60,100,100],goal=[400,100,100])
        env=Environment(world)
        a,first=build_paper_tube(env,cfg,17)
        b,second=build_paper_tube(env,cfg,17)
        self.assertEqual(a.points,b.points)
        self.assertEqual(a.radii,b.radii)
        self.assertEqual(first['waypoints'],second['waypoints'])
        self.assertEqual(a.points[0],world['start'])
        self.assertEqual(a.points[-1],world['goal'])
        self.assertGreaterEqual(a.validate(env,2)['certified_obstacle_and_world_margin'],2)

    def test_runner_outputs_metrics_and_unfinished_status(self):
        from drone_bench.algorithms.planners.reference import build_reference_tube
        cfg=load_scenario('configs/experiments/paper_vb.json')
        env=Environment(load_scenario(cfg['scenario']));tube=build_reference_tube(env)
        cfg['simulation']['duration']=.04
        with tempfile.TemporaryDirectory() as directory:
            frames,history,summary=run(env,tube,cfg,directory,PaperOuter(tube,cfg['outer']),PaperInner(cfg['inner'],cfg['dynamics']))
            self.assertIsNone(summary['error'])
            self.assertFalse(summary['success'])
            self.assertEqual(summary['completed'],0)
            self.assertEqual(len(history),3)
            self.assertEqual(len(frames[0]['positions']),27)
            self.assertLessEqual(summary['max_command_speed'],11+1e-10)
            self.assertTrue((Path(directory)/'metrics.csv').exists())
