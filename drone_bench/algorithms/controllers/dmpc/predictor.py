"""Linear cascaded tracking predictor. Does not modify the physical inner loop."""
import math


class Predictor:
    def __init__(self,horizon,step,blocks,tau_filter,tau_velocity):
        if horizon%blocks:raise ValueError('Horizon must be divisible by move blocks')
        self.N,self.h,self.M=horizon,step,blocks
        self.block_length=horizon//blocks
        self.tf,self.tv=tau_filter,tau_velocity
        self.P,self.V,self.Z=self._sensitivity()

    def advance(self,p,v,z,u,b,h=None):
        h=self.h if h is None else h
        tf,tv=self.tf,self.tv;af=math.exp(-h/tf);av=math.exp(-h/tv)
        if abs(tf-tv)<1e-8:raise ValueError('Predictor time constants too close; use distinct values')
        c=tf/(tf-tv)*(af-av)
        pv=tv*(1-av)
        pz=tf/(tf-tv)*(tf*(1-af)-tv*(1-av))
        pu=h-pv-pz;vb=tv*(1-av);pb=tv*h-tv*tv*(1-av)
        return ([p[k]+pv*v[k]+pz*z[k]+pu*u[k]+pb*b[k] for k in range(3)],
                [av*v[k]+c*z[k]+(1-av-c)*u[k]+vb*b[k] for k in range(3)],
                [af*z[k]+(1-af)*u[k] for k in range(3)])

    def rollout(self,p,v,z,controls,b=None):
        b=b or [0,0,0];positions=[];velocities=[];filters=[]
        for k in range(self.N):
            u=controls[k//self.block_length]
            p,v,z=self.advance(p,v,z,u,b)
            positions.append(p);velocities.append(v);filters.append(z)
        return positions,velocities,filters

    def _sensitivity(self):
        P=[[0.0]*self.M for _ in range(self.N)];V=[r[:] for r in P];Z=[r[:] for r in P]
        for j in range(self.M):
            controls=[[float(i==j),0,0] for i in range(self.M)]
            ps,vs,zs=self.rollout([0,0,0],[0,0,0],[0,0,0],controls)
            for k in range(self.N):P[k][j]=ps[k][0];V[k][j]=vs[k][0];Z[k][j]=zs[k][0]
        return P,V,Z

    def shift(self,controls,steps=1):
        expanded=[list(controls[k//self.block_length]) for k in range(self.N)]
        shifted=expanded[steps:]+[expanded[-1]]*steps
        # Average move blocks, preserving convex norm bounds.
        return [[sum(shifted[min(j*self.block_length+k,self.N-1)][a] for k in range(self.block_length))/self.block_length
                 for a in range(3)] for j in range(self.M)]
