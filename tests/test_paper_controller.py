import math
import unittest
from drone_bench.algorithms.controllers.potentials import repulsive,cohesive
from drone_bench.algorithms.controllers.paper_outer import PaperOuter
from drone_bench.algorithms.controllers.paper_inner import PaperInner
from drone_bench.tubes.model import Tube
from drone_bench.io.benchmark import load_scenario


class PaperControllerTests(unittest.TestCase):
    def test_potential_gradients_match_finite_differences(self):
        cases=[(lambda x:repulsive(x,10,15,300,.01),[8,10.5,13,16]),
               (lambda x:repulsive(x,5,10,250,.01),[3,6,9,11]),
               (lambda x:cohesive(x,100,150,3),[80,110,140,160])]
        for fn,points in cases:
            for x in points:
                eps=1e-4
                numerical=(fn(x+eps)[0]-fn(x-eps)[0])/(2*eps)
                self.assertAlmostEqual(fn(x)[1],numerical,places=6)

    def test_outer_force_symmetry_and_speed_bound(self):
        cfg=load_scenario('configs/experiments/paper_vb.json')['outer']
        tube=Tube([[0,0,0],[200,0,0]],[40,40])
        ctrl=PaperOuter(tube,cfg)
        commands,_=ctrl.commands([[20,-6,0],[20,6,0]],[0,1])
        self.assertLess(commands[0][1],0)
        self.assertGreater(commands[1][1],0)
        self.assertAlmostEqual(commands[0][1],-commands[1][1])
        self.assertLessEqual(math.sqrt(sum(x*x for x in commands[0])),11+1e-10)
        # Cohesion outside ls pulls two agents toward one another.
        commands,_=ctrl.commands([[20,0,0],[140,0,0]],[0,1])
        self.assertGreater(commands[0][0],commands[1][0])

    def test_boundary_repulsion_points_inward(self):
        cfg=load_scenario('configs/experiments/paper_vb.json')['outer']
        ctrl=PaperOuter(Tube([[0,0,0],[200,0,0]],[40,40]),cfg)
        commands,_=ctrl.commands([[20,34,0]],[0])
        self.assertLess(commands[0][1],0)

    def test_inner_zero_error_cancellation_and_saturation(self):
        cfg=load_scenario('configs/experiments/paper_vb.json')
        inner=PaperInner(cfg['inner'],cfg['dynamics'])
        vc=[0,0,0];state=inner.initial_state([50,140,50],vc,vc)
        diagnostic=inner.rhs(0,state,vc,0,True)
        self.assertEqual(diagnostic['error'],[0,0,0])
        self.assertAlmostEqual(diagnostic['unconstrained_input'][2],9.81)
        state[3]=100
        diagnostic=inner.rhs(0,state,vc,0,True)
        self.assertLessEqual(max(abs(x) for x in diagnostic['input']),30)
        self.assertNotEqual(diagnostic['input'],diagnostic['proxy'])

    def test_midpoint_convergence_for_inner_state(self):
        cfg=load_scenario('configs/experiments/paper_vb.json')
        inner=PaperInner(cfg['inner'],cfg['dynamics'])
        vc=[2,1,.5];initial=inner.initial_state([50,140,50],[0,0,0],vc)
        def integrate(dt):
            state=initial[:]
            for i in range(round(.2/dt)):state=inner.step(i*dt,state,vc,0,dt)
            return state
        a,b,ref=integrate(.01),integrate(.005),integrate(.00125)
        e1=math.sqrt(sum((x-y)**2 for x,y in zip(a,ref)))
        e2=math.sqrt(sum((x-y)**2 for x,y in zip(b,ref)))
        self.assertGreater(e1/e2,3.5)
