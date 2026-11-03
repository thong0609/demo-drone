"""Move-blocked velocity-reference DMPC with sequential convex coordination."""
import copy
import ctypes
import math
import time as clock
from ....common.vectors import dot,sub,norm,mul
from .predictor import Predictor
from .reference import TubeReference
from .optimizer import QPSolver,polytope
from .constraints import build,segment_pair_distance


class DMPC:
    def __init__(self,tube,outer_cfg,inner_cfg,cfg):
        self.c,self.outer=cfg,outer_cfg
        self.geometry=TubeReference(tube,outer_cfg)
        self.predictor=Predictor(cfg['horizon'],cfg['prediction_dt'],cfg['move_blocks'],inner_cfg['filter_tau'],cfg['tau_velocity'])
        self.directions,self.norm_factor=polytope()
        self.solver=QPSolver(self._hessian())
        self.geometry.enable_native(self.solver.library)
        ptr=ctypes.POINTER(ctypes.c_double)
        self.solver.library.min_pair_distance.argtypes=[ctypes.c_int,ctypes.c_int,ptr,ptr,ptr,ptr]
        self.solver.library.min_pair_distance.restype=ctypes.c_double
        self.controls={};self.last_commands={};self.previous_measurements={};self.bias={}
        self.next_update=0;self.cycle=0;self.records=[];self.prediction_records=[];self.expectations={}
        self.stats=dict(updates=0,local_solves=0,rejected_solves=0,fallbacks=0,deadline_misses=0,
                        total_solver_seconds=0,max_solver_seconds=0,terminated_infeasible=False,buffer_relaxations=0,max_buffer_relaxation=0)
        self.history=[]

    def _hessian(self):
        p=self.predictor;c=self.c;M=p.M
        scalar=[[0.0]*M for _ in range(M)]
        for j in range(M):
            for l in range(M):
                scalar[j][l]=2*sum((c['position_weight']+c.get('cohesion_weight',0))*p.P[k][j]*p.P[k][l]+c['velocity_weight']*p.V[k][j]*p.V[k][l] for k in range(p.N))
            scalar[j][j]+=2*c['command_weight']
        for j in range(M):
            scalar[j][j]+=2*c['slew_weight']
            if j:
                scalar[j-1][j-1]+=2*c['slew_weight']
                scalar[j][j-1]-=2*c['slew_weight'];scalar[j-1][j]-=2*c['slew_weight']
        matrix=[[scalar[i//3][j//3] if i%3==j%3 else 0 for j in range(3*M)] for i in range(3*M)]
        for r in matrix:r.append(0.0)
        matrix.append([0.0]*(3*M)+[2*c['pair_buffer_weight']])
        return matrix

    def _gradient(self,base_p,base_v,ref_p,ref_v,last):
        p=self.predictor;c=self.c
        result=[2*sum(c['position_weight']*p.P[k][j]*(base_p[k][a]-ref_p[k][a])+c['velocity_weight']*p.V[k][j]*(base_v[k][a]-ref_v[k][a]) for k in range(p.N)) for j in range(p.M) for a in range(3)]
        for a in range(3):result[a]-=2*c['slew_weight']*last[a]
        return result+[0.0]

    def _rollout(self,i,states,controls):
        return self.predictor.rollout(states[i][:3],states[i][3:6],states[i][6:9],controls,self.bias.get(i,[0,0,0]))

    def _validate(self,i,states,controls,positions,velocities,plans,location):
        c=self.c;o=self.outer;minimum_pair=float('inf');minimum_boundary=float('inf')
        previous=states[i][:3]
        for k,(p,v) in enumerate(zip(positions,velocities)):
            # Subdivision of position segments and exact distance for piecewise-linear
            # relative predictions. Does not certify the actual uncertain plant.
            for fraction in (.5,1):
                q=[a+fraction*(b-a) for a,b in zip(previous,p)]
                geo=self.geometry.predicted_geometry(q,key=(i,k),hint=location['segment'])
                minimum_boundary=min(minimum_boundary,geo['clearance'])
                if geo['clearance']<o['rs']+c['boundary_margin']-c['validation_tolerance']:
                    return False,'tube',minimum_pair,minimum_boundary
            if norm(v)>o['vm']-c['speed_margin']+c['validation_tolerance']:return False,'predicted_speed',minimum_pair,minimum_boundary
            previous=p
        others=[j for j in plans if j!=i]
        if others:
            def array(values):return (ctypes.c_double*len(values))(*values)
            own=array([v for point in positions for v in point])
            starts=array([v for j in others for v in states[j][:3]])
            trajectories=array([v for j in others for point in plans[j] for v in point])
            minimum_pair=self.solver.library.min_pair_distance(self.predictor.N,len(others),array(states[i][:3]),own,starts,trajectories)
            if minimum_pair<2*o['rs']+c['pair_hard_margin']-c.get('pair_validation_tolerance',.002):
                return False,'pair',minimum_pair,minimum_boundary
        for j,u in enumerate(controls):
            previous=self.last_commands[i] if j==0 else controls[j-1]
            if norm(u)>o['vm']+1e-6:return False,'command',minimum_pair,minimum_boundary
            if norm(sub(u,previous))>c['command_rate']*self.predictor.h*self.predictor.block_length+c['validation_tolerance']:
                return False,'command_rate',minimum_pair,minimum_boundary
        return True,'accepted',minimum_pair,minimum_boundary

    def _observe(self,t,states,active):
        for i in active:
            current=(t,states[i][:3],states[i][3:6],states[i][6:9])
            old=self.previous_measurements.get(i)
            if old and t>old[0]+1e-9:
                elapsed=t-old[0]
                residual=[(current[2][a]-old[2][a])/elapsed-(old[3][a]-old[2][a])/self.predictor.tv for a in range(3)]
                alpha=1-math.exp(-elapsed/self.c['bias_filter_tau'])
                previous=self.bias.get(i,[0,0,0])
                self.bias[i]=[max(-self.c['bias_limit'],min(self.c['bias_limit'],previous[a]+alpha*(residual[a]-previous[a]))) for a in range(3)]
            self.previous_measurements[i]=current
        due=[key for key in self.expectations if key[0]<=t+1e-8]
        for key in due:
            _,i,horizon=key;predicted=self.expectations.pop(key)
            if i in active:self.prediction_records.append(dict(time=t,agent=i,horizon=horizon,position_error=norm(sub(states[i][:3],predicted))))

    def export_state(self):
        """Only causal controller state; omit logs to keep debug checkpoints small."""
        values={name:getattr(self,name) for name in ('controls','last_commands','previous_measurements','bias','next_update','cycle')}
        values['geometry_cache']=self.geometry.geometry.cache
        return copy.deepcopy(values)

    def import_state(self,values):
        for name in ('controls','last_commands','previous_measurements','bias'):
            setattr(self,name,{int(k):copy.deepcopy(v) for k,v in values[name].items()})
        self.next_update=values['next_update'];self.cycle=values['cycle']
        self.geometry.geometry.cache={int(k):v for k,v in values['geometry_cache'].items()}

    def commands(self,positions,active,*,time=0.0,states=None):
        if states is None:raise ValueError('DMPC requires measured velocity and filter state')
        locations={i:self.geometry.locate(positions[i],i) for i in active}
        self._observe(time,states,active)
        if not active:return {},locations
        if not self.last_commands and time==0:
            self.last_commands={i:list(states[i][6:9]) for i in active}
            self.next_update=self.c['initial_hold']
            return {i:self.last_commands[i] for i in active},locations
        if time+1e-8<self.next_update:return {i:self.last_commands[i] for i in active},locations
        started=clock.perf_counter()
        p=self.predictor;zero=[[0,0,0] for _ in range(p.M)]
        references={i:self.geometry.reference(locations[i],states[i][3:6],p.h,p.N,self.c['cruise_speed'],p.tf+p.tv) for i in active}
        for i in active:
            if i not in self.last_commands:self.last_commands[i]=list(states[i][6:9])
            if i in self.controls:self.controls[i]=p.shift(self.controls[i],round(self.c['update_period']/p.h))
            else:self.controls[i]=[list(self.last_commands[i]) for _ in range(p.M)]
        plans={i:self._rollout(i,states,self.controls[i])[0] for i in active}
        order=active[self.cycle%len(active):]+active[:self.cycle%len(active)]
        statuses=[]
        last_failure=None
        final_valid=False
        for coordination in range(self.c['coordination_rounds']):
            for i in order:
                base_p,base_v,_=self._rollout(i,states,zero)
                gradient=self._gradient(base_p,base_v,*references[i],self.last_commands[i])
                peers=[j for j in active if j!=i]
                if self.c.get('cohesion_weight',0):
                    centroid=([[sum(plans[j][k][a] for j in peers)/len(peers) for a in range(3)] for k in range(p.N)] if peers else references[i][0])
                    for j in range(p.M):
                        for a in range(3):
                            gradient[3*j+a]+=2*self.c['cohesion_weight']*sum(p.P[k][j]*(base_p[k][a]-centroid[k][a]) for k in range(p.N))
                controls=self.controls[i];nominal=plans[i];accepted=False;reason='unknown'
                for iteration in range(self.c['scp_iterations']):
                    rows,bounds=build(p,self.geometry,nominal,base_p,base_v,plans,self.last_commands[i],self.c,self.outer,self.directions,self.norm_factor,i,locations[i],states)
                    solve_start=clock.perf_counter()
                    vector,info=self.solver.solve(gradient,rows,bounds,self.c['qp_sweeps'],self.c['qp_tolerance'])
                    elapsed=clock.perf_counter()-solve_start
                    self.stats['local_solves']+=1;self.stats['total_solver_seconds']+=elapsed;self.stats['max_solver_seconds']=max(self.stats['max_solver_seconds'],elapsed)
                    info['solver_status']=info.pop('status')
                    if vector is None:reason=info['solver_status'];break
                    info['buffer_relaxation']=max(0.0,vector[-1])
                    candidate=[vector[j*3:j*3+3] for j in range(p.M)]
                    predicted,velocities,_=self._rollout(i,states,candidate)
                    valid,reason,minpair,minboundary=self._validate(i,states,candidate,predicted,velocities,plans,locations[i])
                    if info['violation']<=self.c['qp_tolerance'] and valid:
                        controls=candidate;plans[i]=predicted;accepted=True
                        relaxation=max(0.0,vector[-1])
                        self.stats['buffer_relaxations']+=int(relaxation>1e-5)
                        self.stats['max_buffer_relaxation']=max(self.stats['max_buffer_relaxation'],relaxation)
                        break
                    if valid:reason='qp_residual'
                    nominal=predicted
                if not accepted:
                    self.stats['rejected_solves']+=1
                    predicted,velocities,_=self._rollout(i,states,controls)
                    valid,fallback_reason,minpair,minboundary=self._validate(i,states,controls,predicted,velocities,plans,locations[i])
                    if not valid:
                        # A neighbor that updates later may resolve this conflict.
                        # Keep the old plan provisionally inside this negotiation,
                        # never apply it until the coupled final check succeeds.
                        last_failure=(i,reason,fallback_reason,dict(info))
                    else:
                        self.stats['fallbacks']+=1
                self.controls[i]=controls
                statuses.append(dict(time=time,agent=i,round=coordination,status='accepted' if accepted else ('validated_previous' if valid else 'pending_coordination'),reason=reason,
                                     min_predicted_pair=None if math.isinf(minpair) else minpair,min_predicted_boundary=minboundary,**info))
            # Stop negotiation early only when the complete coupled plan is valid.
            final_valid=True
            for i in active:
                predicted,velocities,_=self._rollout(i,states,self.controls[i])
                valid,reason,_,_=self._validate(i,states,self.controls[i],predicted,velocities,plans,locations[i])
                if not valid:
                    final_valid=False
                    last_failure=(i,reason,reason,{})
                    break
            if final_valid:break
        if not final_valid:
            self.stats['terminated_infeasible']=True
            self.records.extend(statuses)
            i,reason,previous,info=last_failure
            self.records.append(dict(time=time,agent=i,status='failed',reason=reason,fallback_reason=previous,**info))
            raise ValueError(f'DMPC no validated coupled plan after {self.c["coordination_rounds"]} rounds: agent={i}, reason={reason}')
        self.last_commands={i:list(self.controls[i][0]) for i in active}
        duration=clock.perf_counter()-started
        self.stats['updates']+=1;self.stats['deadline_misses']+=int(duration>self.c['update_period'])
        self.records.extend(statuses);self.history.append(dict(time=time,solve_seconds=duration,active=len(active)))
        for i in active:
            for seconds in (1,2,3):
                k=round(seconds/p.h)-1
                if k<p.N:self.expectations[(round(time+seconds,8),i,seconds)]=plans[i][k]
        self.cycle+=1;self.next_update=time+self.c['update_period']
        return {i:self.last_commands[i] for i in active},locations
