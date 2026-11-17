import copy
import math
import tempfile
import unittest
from pathlib import Path
from drone_bench.algorithms.controllers.paper_inner import PaperInner
from drone_bench.algorithms.controllers.paper_outer import PaperOuter
from drone_bench.algorithms.planners.minimum_snap import minimum_snap
from drone_bench.io.benchmark import load_scenario,make_payload,load_benchmark,write_json
from drone_bench.tubes.model import Tube
from drone_bench.tubes.polynomial import PolynomialTube


def polynomial_tube(axes,radii):
    from drone_bench.algorithms.planners.minimum_snap import eval_curve
    count=len(radii);parameters=[[s,j/64] for s in range(count) for j in range(64)]+[[count-1,1]]
    points=[eval_curve(axes,s,t) for s,t in parameters]
    rs=[eval_curve([radii],s,t)[0] for s,t in parameters]
    return PolynomialTube(points,rs,dict(centerline_coefficients=axes,radius_coefficients=radii,
                          durations=[1]*count,sample_parameters=parameters))


class PaperFormulaFixTests(unittest.TestCase):
    def test_cone_nearest_surface_and_force_have_axial_component(self):
        tube=Tube([[0,0,0],[100,0,0]],[10,60]);p=[50,30,0]
        boundary=tube.nearest_boundary(p)
        self.assertAlmostEqual(boundary['distance'],math.sqrt(20),places=10)
        for a,b in zip(boundary['point'],[48,34,0]):self.assertAlmostEqual(a,b,places=10)
        cfg=load_scenario('configs/experiments/paper_vb.json')['outer']
        commands,locations=PaperOuter(tube,cfg).commands([p],[0])
        self.assertAlmostEqual(locations[0]['clearance'],math.sqrt(20),places=10)
        self.assertEqual(locations[0]['radial_clearance'],5)
        # The inward normal to y=10+.5x has a positive axial component.
        self.assertGreater(locations[0]['boundary_inward'][0],0)
        self.assertLess(commands[0][1],0)

    def test_continuous_cone_matches_analytic_solution(self):
        axes=[[[100*j/7 for j in range(8)]],[[0]*8],[[0]*8]]
        tube=polynomial_tube(axes,[[10+50*j/7 for j in range(8)]])
        b=tube.nearest_boundary([50,30,0])
        self.assertAlmostEqual(b['distance'],math.sqrt(20),places=7)
        for actual,expected in zip(b['point'],[48,34,0]):self.assertAlmostEqual(actual,expected,places=5)

    def test_curved_surface_against_independent_dense_ring_samples(self):
        # Quadratic parabola (100t,40t^2,0), represented in degree-7 Bernstein.
        axes=[[[100*j/7 for j in range(8)]],[[40*j*(j-1)/(7*6) for j in range(8)]],[[0]*8]]
        tube=polynomial_tube(axes,[[8]*8]);p=[48,15,2]
        b=tube.nearest_boundary(p)
        dense=float('inf')
        # Independent analytic center/tangent; no calls to the query evaluator.
        for j in range(10001):
            t=j/10000;dx,dy,dz=p[0]-100*t,p[1]-40*t*t,p[2]
            axial=(100*dx+80*t*dy)/math.sqrt(10000+6400*t*t)
            radial=math.sqrt(max(0,dx*dx+dy*dy+dz*dz-axial*axial))
            dense=min(dense,math.sqrt(axial*axial+(radial-8)**2))
        self.assertLessEqual(b['distance'],dense+1e-7)
        self.assertLess(dense-b['distance'],1e-4)

    def test_continuous_tangent_and_endpoint_limits(self):
        x,_=minimum_snap([0,40,80],[1,1],15,2)
        y,_=minimum_snap([0,20,0],[1,1],15,2)
        tube=polynomial_tube([x,y,[[0]*8,[0]*8]],[[10]*8,[10]*8])
        left=tube.sample(0,1)[2];right=tube.sample(1,0)[2]
        self.assertLess(math.sqrt(sum((a-b)**2 for a,b in zip(left,right))),1e-8)
        self.assertAlmostEqual(sum(v*v for v in tube.sample(0,0)[2]),1)
        self.assertAlmostEqual(sum(v*v for v in tube.sample(1,1)[2]),1)

    def test_projection_does_not_stay_in_a_stale_segment_window(self):
        tube=Tube([[j,0,0] for j in range(100)],[10]*100)
        outer=PaperOuter(tube,load_scenario('configs/experiments/paper_vb.json')['outer'])
        outer.locate([2,1,0],0)
        q=outer.locate([90,1,0],0)
        self.assertAlmostEqual(q['point'][0],90)

    def test_bounded_basis_and_positive_paper_witness(self):
        cfg=load_scenario('configs/experiments/paper_vb.json');inner=PaperInner(cfg['inner'],cfg['dynamics'])
        for Z in ([0]*9,[1]*9,[-1]*9,[1000]*9):
            self.assertLess(math.sqrt(sum(x*x for x in inner.basis(Z))),.5)
        self.assertTrue(inner.certificate['verified'])
        self.assertAlmostEqual(inner.certificate['minimum'],.65)
        bad=copy.deepcopy(cfg['inner']);bad['basis_norm_bound']=1
        with self.assertRaisesRegex(ValueError,'Eq. 77'):PaperInner(bad,cfg['dynamics'])

    def test_uncertainty_is_a_function_of_network_inputs(self):
        from drone_bench.simulation.dynamics import perturbations
        cfg=load_scenario('configs/experiments/paper_vb.json')['dynamics']
        a=perturbations(0,[0,0,0],[1,2,3],0,cfg)[0]
        b=perturbations(0,[100,200,300],[1,2,3],0,cfg)[0]
        self.assertEqual(a,b)

    def test_polynomial_benchmark_round_trip(self):
        axes=[[[60+40*j/7 for j in range(8)]],[[100]*8],[[100]*8]]
        tube=polynomial_tube(axes,[[10]*8]);world=load_scenario('configs/scenarios/benchmark_v1.json')
        world.update(obstacles=[],start=[60,100,100],goal=[100,100,100])
        payload=make_payload(world,tube)
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'benchmark.json';write_json(path,payload);_,restored=load_benchmark(path)
        self.assertEqual(restored.curve,tube.curve)
        self.assertAlmostEqual(restored.nearest_boundary([80,107,100])['distance'],3,places=8)

    def test_continuous_certificate_catches_a_bulge_between_render_samples(self):
        from drone_bench.environment.world import Environment
        # Endpoints are safe, but the polynomial bows through a sphere.
        axes=[[[60+40*j/7 for j in range(8)]],[[100+50*j*(7-j)/(7*6) for j in range(8)]],[[100]*8]]
        curve=dict(centerline_coefficients=axes,radius_coefficients=[[2]*8],durations=[1])
        tube=PolynomialTube([[60,100,100],[100,100,100]],[2,2],curve)
        world=load_scenario('configs/scenarios/benchmark_v1.json')
        world['obstacles']=[dict(type='sphere',center=[80,112.5,100],radius=4)]
        with self.assertRaisesRegex(ValueError,'Continuous polynomial'):
            tube.validate(Environment(world),2)


if __name__=='__main__':unittest.main()
