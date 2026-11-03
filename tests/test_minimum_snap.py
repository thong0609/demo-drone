import unittest
from drone_bench.algorithms.planners.minimum_snap import minimum_snap,basis_derivative


class MinimumSnapTests(unittest.TestCase):
    def test_waypoint_continuity_and_boundary_derivatives(self):
        coefficients,report=minimum_snap([0,25,40],[1,1],15,2)
        def value(s,t,order=0):return sum(a*b for a,b in zip(coefficients[s],basis_derivative(t,order)))
        self.assertAlmostEqual(value(0,0),0,places=6)
        self.assertAlmostEqual(value(0,1),25,places=6)
        self.assertAlmostEqual(value(1,0),25,places=6)
        self.assertAlmostEqual(value(1,1),40,places=6)
        for order in (1,2):
            self.assertAlmostEqual(value(0,1,order),value(1,0,order),places=6)
            self.assertAlmostEqual(value(0,0,order),0,places=6)
            self.assertAlmostEqual(value(1,1,order),0,places=6)
        self.assertLess(report['equality_residual'],1e-6)
        self.assertEqual(report['corridor_violation'],0)

    def test_convex_hull_corridor_over_continuous_curve(self):
        values=[0,25,40];corridor=8
        coeff,report=minimum_snap(values,[1,1],corridor,2)
        for s in range(2):
            for j in range(1001):
                t=j/1000
                p=sum(a*b for a,b in zip(coeff[s],basis_derivative(t)))
                line=values[s]*(1-t)+values[s+1]*t
                self.assertLessEqual(abs(p-line),corridor+1e-6)
