"""Guidance Eq. 22 and enhanced vector-field law Eqs. 34–35."""
import math
from ...common.vectors import add, sub, mul, dot, norm, unit
from .potentials import repulsive, cohesive


def saturate(v, limit):
    length=norm(v)
    return mul(v,min(1,limit/length)) if length else list(v)


class PaperOuter:
    def __init__(self,tube,cfg):
        self.tube,self.cfg=tube,cfg
        self.lengths=[norm(sub(b,a)) for a,b in zip(tube.points,tube.points[1:])]
        self.arc=[0.0]
        for length in self.lengths: self.arc.append(self.arc[-1]+length)
        self.cache={}

    def locate(self,p,agent):
        q=self.tube.project(p);i=q['segment'];t=q['fraction']
        if 'tangent' not in q:
            q.update(offset=sub(p,q['point']),tangent=mul(sub(self.tube.points[i+1],self.tube.points[i]),1/self.lengths[i]),
                     radius_slope=(self.tube.radii[i+1]-self.tube.radii[i])/self.lengths[i])
        boundary=self.tube.nearest_boundary(p)
        q.update(clearance=boundary['distance'] if q['radial_clearance']>=0 else -boundary['distance'],
                 boundary_point=boundary['point'],boundary_inward=boundary['inward'],
                 arc=q.get('arc',self.arc[i]+t*self.lengths[i]))
        return q

    def commands(self,positions,active,*,time=0.0,states=None):
        cfg=self.cfg; states={i:self.locate(positions[i],i) for i in active}; commands={}
        for i in active:
            q=states[i]
            # Eq. 22 derivative in global coordinates, no frame transport substitution.
            tangent=unit(add(q['tangent'],mul(q['offset'],q['radius_slope']/q['radius'])))
            # When k1*lp >= vm, guidance is exactly saturated irrespective of
            # the nonnegative remaining guidance-curve length (Eqs. 23–24).
            # Otherwise integrate Eq. 22 along the remaining sampled curve.
            speed=cfg['vm']
            if cfg['k1']*cfg['lp'] < cfg['vm']:
                if hasattr(self.tube,'guidance_length'):
                    remaining=self.tube.guidance_length(q)
                else:
                    remaining=0.0
                    offset_per_radius=mul(q['offset'],1/q['radius'])
                    for j in range(q['segment'],len(self.lengths)):
                        dr=self.tube.radii[j+1]-self.tube.radii[j]
                        delta=add(sub(self.tube.points[j+1],self.tube.points[j]),mul(offset_per_radius,dr))
                        remaining+=norm(delta)*(1-q['fraction'] if j==q['segment'] else 1)
                speed=min(cfg['vm'],cfg['k1']*(remaining+cfg['lp']))
            guidance=mul(tangent,speed)
            v=list(guidance)
            if q['clearance']<=0: raise ValueError(f'UAV {i} left tube')
            _,derivative=repulsive(q['clearance'],cfg['rs'],cfg['ra'],cfg['k3'],cfg['delta_t'])
            # Eqs. 34–35: -V_t'(distance)*(p-p_nearest)/distance.
            v=add(v,mul(q['boundary_inward'],-derivative))
            commands[i]=v
        for index,i in enumerate(active):
            for j in active[index+1:]:
                delta=sub(positions[i],positions[j]); distance=norm(delta)
                if distance<1e-10: raise ValueError('Coincident UAVs')
                derivative=0
                if distance < cfg['rs']+cfg['ra']:
                    derivative+=repulsive(distance,2*cfg['rs'],cfg['rs']+cfg['ra'],cfg['k2'],cfg['delta_n'])[1]
                # Set L_i in Eq. 9 includes only distances up to lm.
                if cfg['ls']<=distance<=cfg['lm']:
                    derivative+=cohesive(distance,cfg['ls'],cfg['lm'],cfg['k4'])[1]
                force=mul(delta,-derivative/distance)
                commands[i]=add(commands[i],force);commands[j]=sub(commands[j],force)
        return {i:saturate(v,cfg['vm']) for i,v in commands.items()},states
