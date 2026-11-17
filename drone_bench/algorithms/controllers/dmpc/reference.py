"""Tube geometry adapter and Eq. 22-inspired individual reference curves."""
import ctypes
import bisect
import math
from ....common.vectors import add,sub,mul,dot,norm,unit,cross
from ..paper_outer import PaperOuter


class TubeReference:
    def __init__(self,tube,outer_cfg):
        if hasattr(tube,'curve'):
            raise ValueError('DMPC sampled/native predictor does not support continuous polynomial tubes; use a sampled benchmark explicitly')
        self.tube=tube;self.geometry=PaperOuter(tube,outer_cfg)
        self.arc=self.geometry.arc;self.length=self.arc[-1]
        self.projection_cache={}
        self.native=None
        self.end_tangent=unit(sub(tube.points[-1],tube.points[-2]))

    def enable_native(self,library):
        self.native=library
        ptr=ctypes.POINTER(ctypes.c_double)
        library.project_tube.argtypes=[ctypes.c_int,ptr,ptr,ptr,ptr]
        self.native_points=(ctypes.c_double*(3*len(self.tube.points)))(*[v for p in self.tube.points for v in p])
        self.native_radii=(ctypes.c_double*len(self.tube.radii))(*self.tube.radii)

    def sample(self,s):
        if s>=self.length:
            return add(self.tube.points[-1],mul(self.end_tangent,s-self.length)),self.tube.radii[-1],self.end_tangent,0.0
        i=max(0,min(len(self.arc)-2,bisect.bisect_right(self.arc,s)-1))
        fraction=max(0,(s-self.arc[i])/(self.arc[i+1]-self.arc[i]))
        a,b=self.tube.points[i:i+2];length=self.arc[i+1]-self.arc[i]
        radius=self.tube.radii[i]*(1-fraction)+self.tube.radii[i+1]*fraction
        return add(a,mul(sub(b,a),fraction)),radius,mul(sub(b,a),1/length),(self.tube.radii[i+1]-self.tube.radii[i])/length

    def locate(self,p,agent):
        return self.geometry.locate(p,agent)

    def predicted_geometry(self,p,key=None,hint=None):
        # Full projection avoids cached-neighborhood artifacts for horizon jumps.
        beyond=dot(sub(p,self.tube.points[-1]),self.end_tangent)
        if beyond>=0:
            q=add(self.tube.points[-1],mul(self.end_tangent,beyond))
            return dict(point=q,radius=self.tube.radii[-1],tangent=self.end_tangent,radius_slope=0,
                        clearance=self.tube.radii[-1]-norm(sub(p,q)),exited=True)
        if self.native is not None:
            query=(ctypes.c_double*3)(*p);result=(ctypes.c_double*11)()
            self.native.project_tube(len(self.tube.points),self.native_points,self.native_radii,query,result)
            return dict(point=list(result[3:6]),radius=result[2],tangent=list(result[6:9]),radius_slope=result[9],clearance=result[10],exited=False)
        if key is None:
            q=self.tube.project(p)
        else:
            index=self.projection_cache.get(key,hint or 0)
            window=14
            while True:
                lo=max(0,index-window);hi=min(len(self.arc)-2,index+window)
                q=None
                for j in range(lo,hi+1):
                    a,b=self.tube.points[j:j+2];delta=sub(b,a);length=self.arc[j+1]-self.arc[j]
                    fraction=max(0,min(1,dot(sub(p,a),delta)/(length*length)))
                    point=add(a,mul(delta,fraction));distance=norm(sub(p,point))
                    if q is None or distance<q['distance']:
                        radius=self.tube.radii[j]*(1-fraction)+self.tube.radii[j+1]*fraction
                        q=dict(segment=j,point=point,radius=radius,distance=distance,radial_clearance=radius-distance)
                if (q['segment']==lo and lo>0) or (q['segment']==hi and hi<len(self.arc)-2):
                    window*=2
                else:break
            self.projection_cache[key]=q['segment']
        i=q['segment'];length=self.arc[i+1]-self.arc[i]
        return dict(point=q['point'],radius=q['radius'],tangent=unit(sub(self.tube.points[i+1],self.tube.points[i])),
                    radius_slope=(self.tube.radii[i+1]-self.tube.radii[i])/length,
                    clearance=q['radial_clearance'],exited=False)

    def reference(self,location,v,h,N,cruise,tau):
        offset=mul(location['offset'],1/location['radius'])
        along=max(0,dot(v,location['tangent']))
        points=[];velocities=[];previous=add(location['point'],location['offset'])
        for k in range(N):
            t=(k+1)*h
            advance=cruise*t+(along-cruise)*tau*(1-math.exp(-t/tau))
            center,radius,_,_=self.sample(location['arc']+max(0,advance))
            p=add(center,mul(offset,radius))
            points.append(p);velocities.append(mul(sub(p,previous),1/h));previous=p
        return points,velocities
