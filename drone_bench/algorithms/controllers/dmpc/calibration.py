"""Fit reduced tracking time constant on separate, deterministic excitation data."""
import math
from ..paper_inner import PaperInner
from .predictor import Predictor
from ....common.vectors import norm,sub


def excite(cfg,commands,hold=2.0):
    inner=PaperInner(cfg['inner'],cfg['dynamics']);dt=cfg['simulation']['dt']
    state=inner.initial_state([50,140,50],[0,0,0],commands[0]);data=[]
    for step in range(round(len(commands)*hold/dt)):
        t=step*dt;u=commands[min(len(commands)-1,int(t/hold))]
        new=inner.step(t,state,u,0,dt)
        data.append((t,state[:],u[:],[(b-a)/dt for a,b in zip(state[3:6],new[3:6])]))
        state=new
    return data


def calibrate(cfg):
    train=excite(cfg,[[3,0,0],[7,2,0],[2,-2,1],[-2,3,-1],[5,0,1],[0,0,0]])
    x=[];y=[]
    for t,state,u,acceleration in train:
        if t<.2:continue
        x.extend(a-b for a,b in zip(state[6:9],state[3:6]));y.extend(acceleration)
    # One positive scalar lag for all axes, no intercept; residual is handled
    # online from past measurements and validation is reported independently.
    alpha=sum(a*b for a,b in zip(x,y))/sum(a*a for a in x)
    if alpha<=0:raise ValueError('Positive tracking lag cannot be identified')
    tau=1/alpha
    validation=excite(cfg,[[2,1,0],[5,-1,1],[0,3,0],[4,0,-1],[1,-2,1]],hold=2.4)
    dt=cfg['simulation']['dt'];pred=Predictor(30,.1,10,cfg['inner']['filter_tau'],tau)
    errors={1:[],2:[],3:[]}
    for start in range(0,len(validation)-round(3/dt)-1,25):
        state=validation[start][1];p,v,z=state[:3],state[3:6],state[6:9]
        for j in range(1,round(3/dt)+1):
            p,v,z=pred.advance(p,v,z,validation[start+j-1][2],[0,0,0],h=dt)
            seconds=round(j*dt,8)
            if seconds in errors:errors[int(seconds)].append(norm(sub(p,validation[start+j][1][:3])))
    return dict(tau_velocity=tau,training='12 s separate step excitation; positive least squares fit',
                validation='independent 12 s command profile, known future commands; zero bias; not a robustness certificate',
                errors={str(k):dict(samples=len(v),position_rmse=math.sqrt(sum(x*x for x in v)/len(v)),max_position_error=max(v)) for k,v in errors.items()})
