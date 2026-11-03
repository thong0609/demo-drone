"""Static world and collision queries shared by all algorithms."""
import math
from ..common.vectors import add, sub, mul, norm
from .obstacles import box_sdf

class Environment:
    def __init__(self, config):
        self.config = config
        lo, hi = config['bounds']
        if any(a >= b for a, b in zip(lo, hi)): raise ValueError('Invalid bounds')
        for o in config['obstacles']:
            if o['type'] not in ('box', 'sphere'): raise ValueError('Unknown obstacle type')

    def clearance(self, p):
        """Positive free-space clearance, negative inside obstacles/outside world.

        Min of primitive SDFs and inward world planes; exact free-space distance.
        """
        lo, hi = self.config['bounds']
        distances = [x-a for x, a in zip(p, lo)] + [b-x for x, b in zip(p, hi)]
        for o in self.config['obstacles']:
            distances.append(box_sdf(p, o['min'], o['max']) if o['type'] == 'box'
                             else norm(sub(p, o['center']))-o['radius'])
        return min(distances)

    def segment_free(self, a, b, radius=0, spacing=1):
        """Conservative sampled certificate using the 1-Lipschitz distance bound."""
        if spacing <= 0 or radius < 0: raise ValueError('Invalid spacing/radius')
        n = max(1, math.ceil(norm(sub(b, a))/spacing))
        half_step = norm(sub(b, a))/(2*n)
        return all(self.clearance(add(a, mul(sub(b, a), i/n))) >= radius+half_step
                   for i in range(n+1))


