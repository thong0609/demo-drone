"""Algorithm 1: SDF → enhanced RRT* → minimum-snap centerline and radius."""
import math
from .enhanced_rrt import plan
from .minimum_snap import minimum_snap, eval_curve
from ...common.vectors import norm, sub
from ...tubes.model import Tube


def build_paper_tube(env, cfg, seed):
    path, report=plan(env,cfg,seed)
    durations=[max(0.5,norm(sub(b,a))/60) for a,b in zip(path,path[1:])]
    coefficients=[]; qp=[]
    for axis in range(3):
        c,r=minimum_snap([p[axis] for p in path],durations,cfg['corridor_half_width'],cfg['snap_gamma'])
        coefficients.append(c); qp.append(r)
    re=env.config['tube']['max_radius']
    radius_values=[min(re,env.clearance(p)-cfg['radius_margin']) for p in path]
    radius,rr=minimum_snap(radius_values,durations,15,cfg['snap_gamma'])
    maximum=max(max(c) for c in radius)
    scale=min(1,re/maximum)
    points=[]; radii=[]; parameters=[]
    for s,T in enumerate(durations):
        n=max(30,math.ceil(norm(sub(path[s+1],path[s]))/cfg['sample_spacing']))
        for j in range(n):
            t=j/n
            points.append(eval_curve(coefficients,s,t)); radii.append(eval_curve([radius],s,t)[0]*scale)
            parameters.append([s,t])
    points.append(path[-1]); radii.append(radius_values[-1]*scale); parameters.append([len(durations)-1,1])
    # Conservative uniform scaling preserves the polynomial radius and all its smoothness.
    margin=env.config['tube']['obstacle_margin']
    safe_scale=1.0
    for a,b,ra,rb in zip(points,points[1:],radii,radii[1:]):
        safe_scale=min(safe_scale,(min(env.clearance(a),env.clearance(b))-norm(sub(b,a))/2-margin)/max(ra,rb))
    scale*=min(1,safe_scale)*(1-1e-9)
    radii=[eval_curve([radius],s,t)[0]*scale for s,t in parameters]
    if min(radii) < cfg['radius_floor']: raise RuntimeError(f'Minimum-snap tube too narrow: min={min(radii):.3f}, scale={scale:.3f}; revise route')
    tube=Tube(points,radii); tube.validate(env,margin)
    report.update(centerline_qp=qp,radius_qp=rr,radius_scale=scale,
                  durations=durations,centerline_coefficients=coefficients,
                  radius_coefficients=[[v*scale for v in c] for c in radius],
                  basis='Bernstein degree 7, local parameter [0,1]',
                  note='Post-QP uniform radius scaling enforces continuous segment clearance certificate.')
    return tube,report
