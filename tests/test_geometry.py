import json
from pathlib import Path
import unittest
from drone_bench.environment.world import Environment
from drone_bench.environment.obstacles import box_sdf
from drone_bench.tubes.model import Tube
from drone_bench.algorithms.planners.reference import build_reference_tube

class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.config=json.loads(Path('configs/scenarios/benchmark_v1.json').read_text())
        self.env=Environment(self.config)

    def test_box_distance_inside_corner_and_face(self):
        self.assertAlmostEqual(box_sdf([1,1,1],[0,0,0],[2,2,2]),-1)
        self.assertAlmostEqual(box_sdf([3,3,2],[0,0,0],[2,2,2]),2**.5)
        self.assertEqual(box_sdf([2,1,1],[0,0,0],[2,2,2]),0)

    def test_world_and_sphere_clearance(self):
        self.assertLess(self.env.clearance([-1,10,10]),0)
        self.assertEqual(self.env.clearance([90,420,140]),-35)
        self.assertEqual(self.env.clearance([150,350,100]),-30)

    def test_crossing_obstacle_rejected(self):
        self.assertFalse(self.env.segment_free([100,350,100],[220,350,100]))
        self.assertTrue(self.env.segment_free([30,30,30],[100,30,30],radius=5))

    def test_reference_geometry_and_certificate(self):
        tube=build_reference_tube(self.env)
        self.assertEqual(tube.points[0],self.config['start'])
        for a,b in zip(tube.points[-1],self.config['goal']): self.assertAlmostEqual(a,b)
        self.assertGreaterEqual(tube.validate(self.env,2)['certified_obstacle_and_world_margin'],2)
        self.assertGreaterEqual(min(tube.radii),12)
        # Independently check interpolated swept-ball clearances within every edge.
        for i in range(len(tube.points)-1):
            for t in [.25,.5,.75]:
                p=[a*(1-t)+b*t for a,b in zip(tube.points[i],tube.points[i+1])]
                r=tube.radii[i]*(1-t)+tube.radii[i+1]*t
                self.assertGreaterEqual(self.env.clearance(p)-r,2)

    def test_unsafe_tube_rejected(self):
        with self.assertRaises(ValueError):
            Tube([[100,350,100],[220,350,100]],[10,10]).validate(self.env,2)

    def test_projection(self):
        tube=Tube([[0,0,0],[10,0,0]],[2,4])
        projection=tube.project([5,1,0])
        self.assertEqual(projection['point'],[5,0,0])
        self.assertEqual(projection['radial_clearance'],2)

if __name__=='__main__': unittest.main()
