"""Common swarm integration loop; algorithm-specific laws live in controllers/."""
import copy
import csv
import time
from pathlib import Path
from ..common.vectors import sub, dot, norm, mul
from ..io.benchmark import write_json
from .initialization import initial_positions
from .metrics import measure,summarize


def run(env,tube,cfg,output,outer,inner,initial_states=None):
    start_clock=time.monotonic();out=Path(output);out.mkdir(parents=True,exist_ok=True)
    if initial_states is None:
        positions=initial_positions(tube,cfg['simulation'],cfg['outer']['rs'])
        active=list(range(len(positions)));vc,_=outer.commands(positions,active)
        speed=cfg['simulation']['initial_speed']
        states=[inner.initial_state(p,mul(vc[i],speed/max(norm(vc[i]),1e-10)),vc[i]) for i,p in enumerate(positions)]
    else:
        states=copy.deepcopy(initial_states)
        positions=[s[:3] for s in states]
        active=list(range(len(states)))
    write_json(out/'initial_state.json',{'positions':positions,'states':states,'agent_ids':active})
    completion_times=[None]*len(states);frames=[];history=[];error=None;failure_time=None
    dt=cfg['simulation']['dt'];record_steps=max(1,round(cfg['simulation']['record_interval']/dt))
    direction=tube.sample(tube.spans-1,1)[2] if hasattr(tube,'curve') else sub(tube.points[-1],tube.points[-2])
    final_tangent=mul(direction,1/norm(direction))
    next_update=0
    for step in range(round(cfg['simulation']['duration']/dt)+1):
        t=step*dt
        checkpoint=None
        if hasattr(outer,'export_state') and (t+1e-8>=outer.next_update):
            checkpoint=dict(time=t,states=copy.deepcopy(states),active=list(active),
                            outer=outer.export_state(),completion_times=list(completion_times))
        try:
            commands,locations=outer.commands([s[:3] for s in states],active,time=t,states=states)
            diagnostic={i:inner.rhs(t,states[i],commands[i],i,True) for i in active}
            metrics=measure(t,states,active,locations,commands,diagnostic,env,cfg['dynamics']['input_limit']);history.append(metrics)
            if step%record_steps==0 or not active:
                frames.append({'time':t,'positions':[s[:3] for s in states], 'active':list(active),
                               'velocities':[s[3:6] for s in states],
                               'disturbance_estimates':[diagnostic[i]['disturbance_estimate'] if i in diagnostic else None for i in range(len(states))]})
            if t>=next_update:
                print(f't={t:.1f}s active={len(active)} min_pair={metrics["min_pair_distance"]} min_boundary={metrics["min_boundary_clearance"]}',flush=True)
                next_update+=10
            if not active or step==round(cfg['simulation']['duration']/dt): break
            if cfg['simulation']['stop_on_violation'] and (metrics['min_pair_distance'] is not None and metrics['min_pair_distance']<2*cfg['outer']['rs'] or metrics['min_boundary_clearance'] is not None and metrics['min_boundary_clearance']<cfg['outer']['rs']):
                raise ValueError('Safety constraint violated')
            for i in list(active):
                previous=states[i]
                states[i]=inner.step(t,states[i],commands[i],i,dt)
                before=dot(sub(previous[:3],tube.points[-1]),final_tangent)
                after=dot(sub(states[i][:3],tube.points[-1]),final_tangent)
                if before<0<=after:
                    fraction=-before/(after-before)
                    crossing=[a+fraction*(b-a) for a,b in zip(previous[:3],states[i][:3])]
                    lateral=norm(sub(crossing,tube.points[-1]))
                    if lateral>tube.radii[-1]-cfg['outer']['rs']:
                        raise ValueError(f'UAV {i} crossed outside safe terminal disk')
                    completion_times[i]=t+dt*fraction
                    states[i][:3]=crossing
                    active.remove(i)
        except (ValueError,OverflowError) as exc:
            error=f'{type(exc).__name__}: {exc}';failure_time=t
            if checkpoint is not None:
                checkpoint['error']=error
                write_json(out/'failure_checkpoint.json',checkpoint)
            break
    summary=summarize(history,cfg,completion_times)
    summary.update(error=error,failure_time=failure_time,simulated_until=history[-1]['time'] if history else 0,
                   integration='explicit midpoint inner dynamics; outer command zero-order hold',
                   wall_seconds=time.monotonic()-start_clock,dt=dt)
    summary['boundary_metric']='Numerical Euclidean distance to continuous polynomial lateral surface' if hasattr(tube,'curve') else 'Euclidean distance to lateral frustum surfaces of sampled tube'
    summary['success']=summary['all_completed'] and summary['constraints_passed'] and error is None
    write_json(out/'final_state.json',dict(time=summary['simulated_until'],states=states,active=active,completion_times=completion_times))
    write_json(out/'summary.json',summary);write_json(out/'trajectory.json',{'frames':frames,'summary':summary})
    with (out/'metrics.csv').open('w',newline='') as f:
        if history:
            writer=csv.DictWriter(f,fieldnames=list(history[0]));writer.writeheader();writer.writerows(history)
    return frames,history,summary
