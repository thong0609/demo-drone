import copy
import json
import tempfile
import unittest
from pathlib import Path

from drone_bench.environment.swept import SweptEnvironment
from drone_bench.environment.world import Environment
from drone_bench.algorithms.planners.swept_volume import check_tube, update_tube
from drone_bench.cli.plan_swept_volume import run
from drone_bench.tubes.model import Tube
from drone_bench.algorithms.planners.paper_tube import build_tube_from_path
from drone_bench.algorithms.planners.tube_quality import shape_metrics, shape_check, displacement


def config():
    return json.loads(Path('configs/experiments/swept_volume.json').read_text())


def obstacle(center=(250, 250, 150), velocity=(0, 0, 0), radius=5, timestamp=0):
    return dict(id='test', shape='sphere', center=list(center), velocity=list(velocity),
                radius=radius, timestamp=timestamp)


class SweptGeometryTests(unittest.TestCase):
    def test_static_sphere_capsule_endpoints_and_continuous_crossing(self):
        world = config()['world']
        moving = obstacle((250, 100, 150), (0, 100, 0), 5)
        env = SweptEnvironment(world, [moving], 0, 3)
        # At h=0 and h=3 the obstacle is far away; its continuous sweep crosses p.
        self.assertAlmostEqual(env.clearance([250, 250, 150]), -5)
        self.assertAlmostEqual(env.clearance([260, 250, 150]), 5)
        self.assertAlmostEqual(env.clearance([250, 90, 150]), 5)
        stationary = SweptEnvironment(world, [obstacle()], 0, 3, margin=2)
        self.assertAlmostEqual(stationary.clearance([260, 250, 150]), 3)
        self.assertFalse(env.segment_free([200, 250, 150], [300, 250, 150], spacing=1))

    def test_age_validation_and_immutable_snapshot(self):
        item = obstacle(velocity=(10, 0, 0))
        world = config()['world']
        env = SweptEnvironment(world, [item], 1, 2, max_age=1)
        self.assertEqual(env.capsules[0]['a'], [260, 250, 150])
        self.assertEqual(env.capsules[0]['b'], [280, 250, 150])
        item['center'][0] = 10
        world['start'][0] = 0
        self.assertEqual(env.capsules[0]['a'][0], 260)
        self.assertEqual(env.config['start'][0], 60)
        for now in (-1, 2):
            with self.assertRaises(ValueError):
                SweptEnvironment(world, [obstacle()], now, 2, max_age=1)

    def test_invalid_records_and_union(self):
        c = config()
        for changes in (dict(radius=-1), dict(velocity=[0, float('nan'), 0]),
                        dict(shape='box'), dict(center=[1, 2])):
            item = obstacle(); item.update(changes)
            with self.assertRaises(ValueError):
                SweptEnvironment(c['world'], [item], 0, 3)
        items = [obstacle(), dict(obstacle((350, 250, 150), radius=10), id='second')]
        env = SweptEnvironment(c['world'], items, 0, 3)
        self.assertEqual(env.clearance([350, 250, 150]), -10)
        with self.assertRaises(ValueError):
            SweptEnvironment(c['world'], [items[0], items[0]], 0, 3)
        c['world']['obstacles'] = [dict(type='sphere', center=[100, 100, 100], radius=9)]
        self.assertEqual(SweptEnvironment(c['world'], [], 0, 3).clearance([100, 100, 100]), -9)

    def test_full_tube_not_only_centerline(self):
        c = config()
        env = SweptEnvironment(c['world'], [obstacle((250, 260, 150), radius=2)], 0, 3)
        tube = Tube([[240, 250, 150], [260, 250, 150]], [12, 12])
        self.assertGreater(env.clearance([250, 250, 150]), 0)
        with self.assertRaises(ValueError):
            check_tube(tube, env, 0, 8)

    def test_certificate_detects_between_sample_collision(self):
        c = config()
        env = SweptEnvironment(c['world'], [obstacle(radius=1)], 0, 3)
        tube = Tube([[200, 250, 150], [300, 250, 150]], [8, 8])
        self.assertTrue(all(env.clearance(p) > 10 for p in tube.points))
        with self.assertRaises(ValueError):
            check_tube(tube, env, 2, 8)


class SweptPlannerTests(unittest.TestCase):
    def test_same_waypoints_balancing_removes_observed_overshoot(self):
        c=config()
        path=[[60,250,150],[295.0337036479149,256.85997260733683,68.3681319835239],
              [384.1657428013534,271.16623799828193,129.20975303663786],[440,250,150]]
        env=Environment(c['world'])
        old,_=build_tube_from_path(env,c['planner'],path)
        cfg=dict(c['planner'],duration_policy='balanced',max_length_inflation=1.1)
        new,_=build_tube_from_path(env,cfg,path)
        a,b=shape_metrics(old,path),shape_check(new,cfg,path)
        self.assertGreater(a['backward_distance_along_start_goal'],10)
        self.assertLess(b['backward_distance_along_start_goal'],1e-6)
        self.assertLess(b['length'],a['length']*.9)
        self.assertEqual(displacement(new,new)['maximum'],0)
        with self.assertRaises(ValueError):shape_check(old,cfg,path)
        snapshot=SweptEnvironment(c['world'],[],0,3)
        replacement,report=update_tube(snapshot,dict(cfg,reuse_waypoints=True),17,old)
        self.assertTrue(report['accepted'],report)
        self.assertEqual(report['action'],'resmooth_previous')
        self.assertTrue(report['previous_check']['certified'])
        self.assertFalse(report['previous_check']['quality_accepted'])

    def test_reuse_waypoints_resmooths_then_certifies(self):
        c=config();cfg=dict(c['planner'],duration_policy='balanced',reuse_waypoints=True,max_length_inflation=1.1)
        path=[c['world']['start'],[250,250,100],c['world']['goal']]
        previous,_=build_tube_from_path(Environment(c['world']),cfg,path)
        # New obstacle affects the tube radius but not the waypoint segments.
        env=SweptEnvironment(c['world'],[obstacle((250,280,100),radius=3)],0,3)
        tube,report=update_tube(env,cfg,17,previous)
        self.assertTrue(report['accepted'],report)
        self.assertEqual(report['action'],'resmooth_previous')
        for actual,expected in zip(report['planning']['waypoints'],path):
            for a,b in zip(actual,expected):self.assertAlmostEqual(a,b,places=8)
        check_tube(tube,env,2,8)

    def test_blocked_start_including_future_sweep(self):
        c = config()
        start = c['world']['start']
        item = obstacle((start[0], start[1]+40, start[2]), (0, -20, 0))
        env = SweptEnvironment(c['world'], [item], 0, 3)
        tube, report = update_tube(env, c['planner'], 17)
        self.assertIsNone(tube)
        self.assertEqual(report['status'], 'start_blocked')
        self.assertFalse(report['accepted'])
        self.assertIn('elapsed_seconds', report)

    def test_no_route_does_not_return_old_tube(self):
        c = config()
        # A sphere separating the entire bounded world, with free anchors.
        c['world']['bounds'] = [[0, 200, 100], [500, 300, 200]]
        env = SweptEnvironment(c['world'], [obstacle(radius=80)], 0, 3)
        previous = Tube([c['world']['start'], c['world']['goal']], [8, 8])
        c['planner']['iterations'] = 30
        tube, report = update_tube(env, c['planner'], 17, previous)
        self.assertIsNone(tube)
        self.assertFalse(report['previous_check']['certified'])
        self.assertEqual(report['status'], 'planning_failed')

    def test_replan_certify_retain_and_export(self):
        c = config()
        before = copy.deepcopy(c)
        with tempfile.TemporaryDirectory() as directory:
            summary = run(c, directory)
            self.assertEqual(summary['accepted'], 4)
            self.assertEqual(summary['replanned'], 1)
            self.assertEqual(summary['retained'], 3)
            self.assertFalse(summary['realtime_demonstrated'])
            self.assertTrue((Path(directory) / 'report.html').exists())
            first = json.loads((Path(directory) / 'update_000.json').read_text())
            self.assertFalse(first['report']['previous_check']['certified'])
            self.assertGreaterEqual(first['report']['certificate']['certified_obstacle_and_world_margin'], 2-1e-8)
            # Reconstruct the polynomial and independently rerun its certificate.
            from drone_bench.tubes.polynomial import PolynomialTube
            g = first['tube']
            tube = PolynomialTube(g['points'], g['radii'], g['curve'])
            env = SweptEnvironment(c['world'], first['report']['observations'], 0, c['horizon'], c['prediction_margin'])
            tube.validate(env, 2)
            candidate, report = update_tube(env, c['planner'], 17, tube, deadline_seconds=1e-12)
            self.assertIs(candidate, tube)
            self.assertTrue(report['deadline_missed'])
            self.assertFalse(report['live_activation_approved'])
        self.assertEqual(c, before)


class DirectionEvaluationTests(unittest.TestCase):
    def test_direction_cases_cover_axes_and_keep_base_unchanged(self):
        from drone_bench.cli.evaluate_swept_volume import cases
        c=config();original=copy.deepcopy(c);matrix=dict(cases(c))
        self.assertEqual(len(matrix),17)
        for name,axis,sign in [('direction_y_negative',1,-1),('direction_y_positive',1,1),
                               ('direction_x_positive',0,1),('direction_x_negative',0,-1),
                               ('direction_z_negative',2,-1),('direction_z_positive',2,1)]:
            item=matrix[name]['obstacles'][0]
            self.assertEqual(item['velocity'][axis],sign*20)
            self.assertEqual(item['radius'],5)
            center=[p+3*v for p,v in zip(item['center'],item['velocity'])]
            self.assertEqual(center,[250,250,150])
        self.assertEqual(len(matrix['direction_converging']['obstacles']),3)
        self.assertEqual(c,original)

    def test_report_keeps_each_case_seed_mode_and_failure_replay(self):
        import re
        from unittest.mock import patch
        from drone_bench.cli.evaluate_swept_volume import cases,evaluate,MODES
        c=config();matrix=[entry for entry in cases(c) if entry[0] in ('direction_y_negative','blocked')]
        for _,item in matrix:item['planner'].update(iterations=12,goal_bias=1,step=60)
        with tempfile.TemporaryDirectory() as directory:
            with patch('drone_bench.cli.evaluate_swept_volume.cases',return_value=matrix):
                evaluate(c,directory,seeds=(17,))
            out=Path(directory);report=(out/'report.html').read_text()
            for name,_ in matrix:
                self.assertIn('id="'+name+'"',report)
                for mode in MODES:
                    target=out/'replays'/name/'seed_17'/(mode+'.html')
                    self.assertTrue(target.exists())
                    payload=json.loads(re.search(r'<script id="replay-data" type="application/json">(.*?)</script>',target.read_text(),re.S)[1])
                    self.assertEqual(len(payload['updates']),2)
                    for u in payload['updates']:
                        self.assertIn('snapshot',u['report'])
                        self.assertIn('common_horizon_certified',u['report'])
            for target in re.findall(r'href="([^"]+)"',report):
                if not target.startswith('#'):self.assertTrue((out/target).exists(),target)
            details=json.loads((out/'details.json').read_text())
            self.assertEqual(len(details),24)
            self.assertTrue(any(not d['row']['common_horizon_certified'] for d in details))


if __name__ == '__main__':
    unittest.main()
