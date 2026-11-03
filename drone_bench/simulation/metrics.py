"""Common metrics, measured on every simulation step, with explicit exit semantics."""
import math
from ..common.vectors import sub, norm


def measure(t,states,active,locations,commands,diagnostics,env,input_limits):
    pairs=[norm(sub(states[i][:3],states[j][:3])) for n,i in enumerate(active) for j in active[n+1:]]
    return dict(time=t,active=len(active),completed=len(states)-len(active),
                min_pair_distance=min(pairs) if pairs else None,
                max_pair_distance=max(pairs) if pairs else None,
                min_boundary_clearance=min((locations[i]['clearance'] for i in active),default=None),
                min_obstacle_clearance=min((env.clearance(states[i][:3]) for i in active),default=None),
                max_speed=max((norm(states[i][3:6]) for i in active),default=0),
                max_command_speed=max((norm(commands[i]) for i in active),default=0),
                max_input=max((max(abs(x) for x in diagnostics[i]['input']) for i in active),default=0),
                max_input_ratio=max((max(abs(x)/limit for x,limit in zip(diagnostics[i]['input'],input_limits)) for i in active),default=0),
                tracking_rmse=math.sqrt(sum(sum(x*x for x in diagnostics[i]['tracking_error']) for i in active)/max(1,3*len(active))),
                disturbance_rmse=math.sqrt(sum(sum((a-b)**2 for a,b in zip(diagnostics[i]['disturbance_estimate'],diagnostics[i]['observer_residual_truth'])) for i in active)/max(1,3*len(active))))


def summarize(history,cfg,completion_times):
    def minimum(key): return min((r[key] for r in history if r[key] is not None),default=None)
    def maximum(key): return max((r[key] for r in history if r[key] is not None),default=None)
    outer=cfg['outer'];dt=cfg['simulation']['dt']
    violations={
        'pair_safety':sum(r['min_pair_distance'] is not None and r['min_pair_distance']<2*outer['rs']-1e-6 for r in history),
        'boundary_safety':sum(r['min_boundary_clearance'] is not None and r['min_boundary_clearance']<outer['rs']-1e-6 for r in history),
        'cohesion':sum(r['max_pair_distance'] is not None and r['max_pair_distance']>outer['lm']+1e-6 for r in history),
        'actual_speed':sum(r['max_speed']>outer['vm']+1e-6 for r in history),
        'obstacle_safety':sum(r['min_obstacle_clearance'] is not None and r['min_obstacle_clearance']<outer['rs']-1e-6 for r in history),
        'input_saturation':sum(r['max_input_ratio']>1+1e-6 for r in history),
        'command_speed':sum(r['max_command_speed']>outer['vm']+1e-6 for r in history)}
    return dict(completed=sum(t is not None for t in completion_times),total=len(completion_times),
                completion_times=completion_times,finish_time=max((t for t in completion_times if t is not None),default=None),
                min_pair_distance=minimum('min_pair_distance'),max_pair_distance=maximum('max_pair_distance'),
                min_boundary_clearance=minimum('min_boundary_clearance'),min_obstacle_clearance=minimum('min_obstacle_clearance'),
                max_speed=maximum('max_speed'),max_command_speed=maximum('max_command_speed'),max_input=maximum('max_input'),
                violation_steps=violations,violation_duration_estimates={k:v*dt for k,v in violations.items()},
                constraints_passed=not any(violations.values()),all_completed=all(t is not None for t in completion_times),
                metrics_scope='Active UAVs only; completed UAVs exit through terminal disk and leave the domain. Metrics sampled each dt, not a continuous-time guarantee.')
