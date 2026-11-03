import math
import unittest
from drone_bench.algorithms.controllers.dmpc.optimizer import QPSolver,polytope
from drone_bench.algorithms.controllers.dmpc.predictor import Predictor
from drone_bench.algorithms.controllers.dmpc.constraints import segment_pair_distance


class DMPCNumericalTests(unittest.TestCase):
    def test_qp_known_coupled_optimum(self):
        solver=QPSolver([[2,0],[0,2]])
        x,info=solver.solve([-4,-4],[[1,1],[-1,0],[0,-1]],[2,0,0],tolerance=1e-8)
        self.assertEqual(info['status'],'optimal')
        self.assertAlmostEqual(x[0],1,places=7);self.assertAlmostEqual(x[1],1,places=7)

    def test_infeasible_qp_is_reported(self):
        solver=QPSolver([[2]])
        _,info=solver.solve([0],[[1],[-1]],[0,-1],max_sweeps=50)
        self.assertEqual(info['status'],'residual_exceeded')
        self.assertGreater(info['violation'],.5)

    def test_inner_polytope_respects_euclidean_speed(self):
        directions,scale=polytope();solver=QPSolver([[2,0,0],[0,2,0],[0,0,2]])
        self.assertTrue(0<scale<1)
        for target in [[100,100,100],[100,30,0],[0,100,44],[60,-44,32]]:
            x,info=solver.solve([-2*a for a in target],directions,[11*scale]*len(directions),max_sweeps=500,tolerance=1e-7)
            self.assertLessEqual(math.sqrt(sum(a*a for a in x)),11+1e-5)

    def test_exact_cascade_matches_fine_euler(self):
        predictor=Predictor(30,.1,10,.15,.4)
        p,v,z=[1,2,3],[2,0,1],[3,1,-1];u=[4,2,0];b=[.1,-.2,.3]
        exact=predictor.advance(p,v,z,u,b,h=.1)
        dt=.00001
        for _ in range(10000):
            pnew=[p[k]+dt*v[k] for k in range(3)]
            vnew=[v[k]+dt*((z[k]-v[k])/.4+b[k]) for k in range(3)]
            znew=[z[k]+dt*(u[k]-z[k])/.15 for k in range(3)]
            p,v,z=pnew,vnew,znew
        for truth,estimate in zip(exact,[p,v,z]):
            for a,c in zip(truth,estimate):self.assertAlmostEqual(a,c,places=4)

    def test_condensed_sensitivities_predict_same_rollout(self):
        p=Predictor(30,.1,10,.15,.4)
        initial=[[5,6,7],[2,1,0],[1,2,3]];bias=[.2,0,-.1]
        controls=[[math.sin(j),math.cos(j),j/10] for j in range(10)]
        ps,vs,_=p.rollout(*initial,controls,bias)
        basep,basev,_=p.rollout(*initial,[[0,0,0]]*10,bias)
        for k in range(30):
            for a in range(3):
                self.assertAlmostEqual(ps[k][a],basep[k][a]+sum(p.P[k][j]*controls[j][a] for j in range(10)),places=10)
                self.assertAlmostEqual(vs[k][a],basev[k][a]+sum(p.V[k][j]*controls[j][a] for j in range(10)),places=10)

    def test_between_knot_collision_is_detected(self):
        self.assertEqual(segment_pair_distance([-1,0,0],[1,0,0],[0,-1,0],[0,1,0]),0)

class DMPCIntegrationTests(unittest.TestCase):
    def setup_controller(self,tube=None):
        from drone_bench.io.benchmark import load_scenario,load_benchmark
        from drone_bench.algorithms.controllers.dmpc.controller import DMPC
        cfg=load_scenario('configs/experiments/paper_vb.json')
        settings=load_scenario('configs/experiments/paper_vb_dmpc.json')
        env,existing=load_benchmark(settings['benchmark'])
        controller=DMPC(tube or existing,cfg['outer'],cfg['inner'],settings['dmpc'])
        return env,existing,cfg,settings,controller

    def test_native_projection_matches_python_geometry(self):
        _,tube,_,_,controller=self.setup_controller()
        for index in (20,80,140,200):
            p=[a+b for a,b in zip(tube.points[index],[1,-2,3])]
            native=controller.geometry.predicted_geometry(p)
            expected=tube.project(p)
            self.assertAlmostEqual(native['clearance'],expected['radial_clearance'],places=9)
            for a,b in zip(native['point'],expected['point']):self.assertAlmostEqual(a,b,places=9)

    def test_outer_holds_command_between_updates_and_keeps_input_state(self):
        import copy
        from drone_bench.io.benchmark import load_scenario
        _,_,_,settings,controller=self.setup_controller()
        states=load_scenario(settings['initial_state'])['states'][:1]
        original=copy.deepcopy(states);positions=[s[:3] for s in states]
        initial,_=controller.commands(positions,[0],time=0,states=states)
        self.assertEqual(initial[0],states[0][6:9])
        first,_=controller.commands(positions,[0],time=.02,states=states)
        held,_=controller.commands(positions,[0],time=.04,states=states)
        self.assertEqual(first,held)
        self.assertEqual(controller.stats['updates'],1)
        self.assertEqual(states,original)
        self.assertLessEqual(math.sqrt(sum(v*v for v in first[0])),11+1e-6)

    def test_infeasible_coupled_plan_is_not_applied(self):
        import copy
        from drone_bench.io.benchmark import load_scenario
        _,_,_,settings,controller=self.setup_controller()
        states=load_scenario(settings['initial_state'])['states'][:2]
        states[1]=copy.deepcopy(states[0])
        positions=[s[:3] for s in states]
        controller.commands(positions,[0,1],time=0,states=states)
        with self.assertRaisesRegex(ValueError,'no validated coupled plan'):
            controller.commands(positions,[0,1],time=.02,states=states)
        self.assertTrue(controller.stats['terminated_infeasible'])

class DMPCFixRegressionTests(unittest.TestCase):
    def test_closest_interior_cut_rejects_endpoint_only_false_safe_path(self):
        from drone_bench.algorithms.controllers.dmpc.constraints import collision_cuts,row
        class LinearPredictor:
            N=1
            M=1
            h=1
            P=[[1.0]]
        cuts=list(collision_cuts(LinearPredictor(),[[8,0,0]],[[0,0,0]],[[0,8,0]],[-8,0,0],[0,-8,0]))
        self.assertEqual(len(cuts),2)
        self.assertAlmostEqual(cuts[1][-1],.5)
        kernel,direction,peer,base,_,_=cuts[1]
        coefficients=row(kernel,direction)
        candidate=[8,0,0,0]
        lhs=sum(a*b for a,b in zip(coefficients,candidate))
        rhs=sum(a*b for a,b in zip(direction,peer))-10.5-sum(a*b for a,b in zip(direction,base))
        self.assertGreater(lhs,rhs)

    def test_preferred_buffer_can_relax_without_relaxing_physical_safety(self):
        import copy
        from drone_bench.io.benchmark import load_scenario,load_benchmark
        from drone_bench.algorithms.controllers.dmpc.controller import DMPC
        settings=load_scenario('configs/experiments/paper_vb_dmpc.json');cfg=load_scenario(settings['base_config'])
        _,tube=load_benchmark(settings['benchmark'])
        initial=load_scenario(settings['initial_state'])['states'][4]
        states=[copy.deepcopy(initial),copy.deepcopy(initial)]
        states[1][1]+=10.3
        for state in states:state[3:9]=[0.0]*6
        ctrl=DMPC(tube,cfg['outer'],cfg['inner'],settings['dmpc'])
        positions=[s[:3] for s in states]
        ctrl.commands(positions,[0,1],time=0,states=states)
        commands,_=ctrl.commands(positions,[0,1],time=.02,states=states)
        self.assertEqual(len(commands),2)
        self.assertGreater(ctrl.stats['max_buffer_relaxation'],.05)
        self.assertLessEqual(ctrl.stats['max_buffer_relaxation'],.4+1e-5)
        self.assertFalse(ctrl.stats['terminated_infeasible'])

    def test_checkpoint_replays_same_outer_update(self):
        import copy,json
        from drone_bench.io.benchmark import load_scenario,load_benchmark
        from drone_bench.algorithms.controllers.dmpc.controller import DMPC
        settings=load_scenario('configs/experiments/paper_vb_dmpc.json');cfg=load_scenario(settings['base_config'])
        _,tube=load_benchmark(settings['benchmark'])
        original=DMPC(tube,cfg['outer'],cfg['inner'],settings['dmpc'])
        states=load_scenario(settings['initial_state'])['states'][:1];positions=[s[:3] for s in states]
        original.commands(positions,[0],time=0,states=states)
        original.commands(positions,[0],time=.02,states=states)
        snapshot=json.loads(json.dumps(original.export_state()))
        restored=DMPC(tube,cfg['outer'],cfg['inner'],settings['dmpc']);restored.import_state(snapshot)
        expected,_=original.commands(positions,[0],time=.22,states=states)
        actual,_=restored.commands(positions,[0],time=.22,states=states)
        for a,b in zip(expected[0],actual[0]):self.assertAlmostEqual(a,b,places=10)
