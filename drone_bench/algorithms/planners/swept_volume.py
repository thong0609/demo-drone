"""Planner-only full-route baseline on swept-volume snapshots.

No swarm execution or safe handover is claimed. A retained tube is re-certified
against the NEW snapshot. Accepted means geometry certified, not real-time safe.
"""
import time
import math
from .paper_tube import build_paper_tube, build_tube_from_path
from .tube_quality import shape_check, displacement
from ...common.vectors import norm, sub


def check_tube(tube, env, margin, radius_floor):
    # Cheap necessary checks prevent deep recursive certification of obviously
    # blocked curves. Lack of certificate is not a proof of physical collision.
    if min(tube.radii) < radius_floor:
        raise ValueError('Tube below requested radius floor')
    samples = [(p, r) for p, r in zip(tube.points, tube.radii)]
    if hasattr(tube, 'curve'):
        # Bernstein hull bounds additionally ensure radius throughout each span.
        for control in tube.curve['radius_coefficients']:
            if min(control) < radius_floor or max(control) > env.config['tube']['max_radius']:
                raise ValueError('Continuous radius bounds not certified')
    elif max(tube.radii) > env.config['tube']['max_radius']:
        raise ValueError('Tube above maximum radius')
    if any(env.clearance(p) < r + margin - 1e-8 for p, r in samples):
        raise ValueError('Sample swept-ball containment check failed')
    return tube.validate(env, margin)


def update_tube(env, cfg, seed, previous=None, deadline_seconds=1.):
    if not math.isfinite(deadline_seconds) or deadline_seconds <= 0:
        raise ValueError('Deadline must be positive')
    started = time.perf_counter()
    result = dict(status=None, accepted=False, action=None, snapshot=env.describe(),
                  previous_check=None, planning=None, certificate=None,attempts=[],
                  scope='planner-only; fixed anchors; no swarm handover')
    tube = None
    margin = env.config['tube']['obstacle_margin']
    try:
        for name in ('start', 'goal'):
            clearance = env.clearance(env.config[name])
            if clearance < 0:
                result.update(status=name + '_blocked', error='Anchor inside swept/static forbidden region')
                return None, result
            if clearance < cfg['radius_floor'] + margin:
                result.update(status=name + '_too_narrow', error='Anchor cannot contain requested tube ball')
                return None, result
        if previous is not None:
            if any(norm(sub(p, q)) > 1e-8 for p, q in (
                    (previous.points[0], env.config['start']),
                    (previous.points[-1], env.config['goal']))):
                raise ValueError('Previous tube anchors differ from requested anchors')
            try:
                certificate = check_tube(previous, env, margin, cfg['radius_floor'])
                if cfg.get('max_length_inflation') is not None:
                    path=([previous.sample(s,0)[0] for s in range(previous.spans)]
                          +[previous.sample(previous.spans-1,1)[0]]) if hasattr(previous,'curve') else previous.points
                    try:
                        result['quality']=shape_check(previous,cfg,path)
                    except ValueError as error:
                        result['previous_check']=dict(certified=True,quality_accepted=False,reason=str(error))
                        raise
                result.update(status='retained', accepted=True, action='retain', certificate=certificate,
                              previous_check=dict(certified=True))
                tube = previous
                return tube, result
            except ValueError as error:
                if result['previous_check'] is None:
                    result['previous_check'] = dict(certified=False, reason=str(error))
        if previous is not None and cfg.get('reuse_waypoints') and hasattr(previous,'curve'):
            path=[previous.sample(s,0)[0] for s in range(previous.spans)]+[previous.sample(previous.spans-1,1)[0]]
            try:
                candidate,planning=build_tube_from_path(env,cfg,path,dict(algorithm='reuse_waypoints'))
                quality=shape_check(candidate,cfg,path)
                certificate=check_tube(candidate,env,margin,cfg['radius_floor'])
                result.update(planning=planning,quality=quality,certificate=certificate,
                              status='replanned',accepted=True,action='resmooth_previous',
                              change=displacement(previous,candidate))
                result['attempts'].append(dict(method='reuse_waypoints',accepted=True))
                return candidate,result
            except (ValueError,RuntimeError,ArithmeticError) as error:
                result['attempts'].append(dict(method='reuse_waypoints',accepted=False,error=str(error)))
        candidate, planning = build_paper_tube(env, cfg, seed)
        result['planning'] = planning
        result['quality']=shape_check(candidate,cfg,planning['waypoints'])
        result['certificate'] = check_tube(candidate, env, margin, cfg['radius_floor'])
        if previous is not None:result['change']=displacement(previous,candidate)
        tube = candidate
        result.update(status='replanned', accepted=True, action='replace')
    except (RuntimeError, ValueError, ArithmeticError) as error:
        result.update(status='planning_failed', error=str(error))
    finally:
        elapsed = time.perf_counter() - started
        result.update(elapsed_seconds=elapsed, deadline_seconds=deadline_seconds,
                      deadline_missed=elapsed > deadline_seconds,
                      timing_policy='measured after completion; not a hard interrupt',
                      live_activation_approved=False)
    return tube, result
