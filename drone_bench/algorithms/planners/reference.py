"""Deterministic reference route, not the paper planner."""
import math
from ...common.vectors import add, sub, mul, norm
from ...tubes.model import Tube

def build_reference_tube(env):
    c = env.config; settings = c['tube']; n = settings['samples']
    if n < 3: raise ValueError('At least 3 samples required')
    points = []
    for i in range(n):
        t=i/(n-1)
        p=add(c['start'],mul(sub(c['goal'],c['start']),t))
        p[1]+=settings['lateral_amplitude']*math.sin(2*math.pi*t)
        p[2]+=settings['vertical_amplitude']*math.sin(math.pi*t)
        points.append(p)
    # Global discretization reserve also covers differing adjacent radii.
    max_step=max(norm(sub(b,a)) for a,b in zip(points,points[1:]))
    reserve=2*max_step
    radii=[min(settings['max_radius'], env.clearance(p)-settings['obstacle_margin']-reserve) for p in points]
    if min(radii)<settings['min_radius']: raise ValueError('Route too narrow; change route, not safety thresholds')
    tube=Tube(points,radii)
    tube.validate(env,settings['obstacle_margin'])
    return tube
