"""Adaptive velocity loop, Eqs. 41–61 and adaptive gain after Eq. 66.

State: p,v,zeta,Lambda,e_hat,xi,kappa_hat, actor weights, critic weights.
Both input models are retained: physical hard saturation (4), tanh proxy (5).
"""
import math
from ...simulation.dynamics import known_dynamics,perturbations


class PaperInner:
    def __init__(self,cfg,dynamics):
        self.c,self.d=cfg,dynamics
        self.features=len(cfg['rbf_centers'])+1
        self.weight_size=3*self.features

    def initial_state(self,p,v,vc):
        # Eq. 41: zeta(0)=vc(0). xi initialized to make D_hat(0)=0.
        error=[v[k]-vc[k] for k in range(3)]
        return list(p)+list(v)+list(vc)+[0.0]*3+list(error)+[-self.c['C'][k]*error[k] for k in range(3)]+[0.0]*3+[0.0]*(2*self.weight_size)

    def rhs(self,t,state,vc,agent, diagnostics=False):
        c,d=self.c,self.d; size=self.weight_size; count=self.features
        p,v,zeta,aux,ehat,xi,khat=[state[j:j+3] for j in range(0,21,3)]
        wa=state[21:21+size];wc=state[21+size:21+2*size]
        dzeta=[(vc[k]-zeta[k])/c['filter_tau'] for k in range(3)]
        error=[v[k]-zeta[k]-aux[k] for k in range(3)]
        prediction=[error[k]-ehat[k] for k in range(3)]
        Z=[x/s for x,s in zip(v+zeta+dzeta,c['feature_scales'])]
        basis=[1.0]+[math.exp(-sum((z-center)**2 for z in Z)/(2*c['rbf_width']**2)) for center in c['rbf_centers']]
        actor=[sum(wa[j*3+k]*basis[j] for j in range(count)) for k in range(3)]
        critic=[sum(wc[j*3+k]*basis[j] for j in range(count)) for k in range(3)]
        en=math.sqrt(sum(x*x for x in error)); na=math.sqrt(sum(x*x for x in wa));nc=math.sqrt(sum(x*x for x in wc))
        wbar=[na,na*nc,nc]; robust=sum(x*y for x,y in zip(wbar,khat))
        estimated_d=[xi[k]+c['C'][k]*error[k] for k in range(3)]
        f=known_dynamics(v,d); g=1/d['mass']
        H=[-(c['K'][k]*error[k]+g*g/2*error[k]+f[k]+actor[k]+robust*math.tanh(robust*error[k]/c['robust_smoothing'])+estimated_d[k]+c['Ka']*aux[k])/g for k in range(3)]
        proxy=[limit*math.tanh(h/limit) for h,limit in zip(H,d['input_limit'])]
        u=[max(-limit,min(limit,h)) for h,limit in zip(H,d['input_limit'])]
        daux=[-c['Ka']*aux[k]-g*(H[k]-proxy[k]) for k in range(3)]
        model=[f[k]+g*u[k]+actor[k]+estimated_d[k]-daux[k] for k in range(3)]
        dehat=[c['P'][k]*prediction[k]+model[k] for k in range(3)]
        dxi=[prediction[k]-c['C'][k]*model[k] for k in range(3)]
        gamma=[prediction[k]+en*critic[k] for k in range(3)]
        dwa=[c['gamma_a']*(basis[j]*(gamma[k]+error[k])-c['eta1']*wa[j*3+k]) for j in range(count) for k in range(3)]
        dwc=[c['gamma_c']*(en*basis[j]*actor[k]-c['eta2']*wc[j*3+k]) for j in range(count) for k in range(3)]
        dkhat=[(en*wbar[k]-c['eta3']*khat[k])/c['Q'] for k in range(3)]
        unknown,disturbance=perturbations(t,p,v,agent,d)
        dv=[f[k]+g*u[k]+unknown[k]+disturbance[k] for k in range(3)]
        result=list(v)+dv+dzeta+daux+dehat+dxi+dkhat+dwa+dwc
        if not all(math.isfinite(x) for x in result): raise ValueError('Nonfinite adaptive state')
        if diagnostics:
            # Observable model residual = Delta - zeta_dot - estimated actor + d.
            # This includes current NN parameter error; optimal W* and exact D
            # in Eq. 48 are unknown, so this is not an oracle for that D.
            truth=[unknown[k]-dzeta[k]-actor[k]+disturbance[k] for k in range(3)]
            return dict(input=u,unconstrained_input=H,proxy=proxy,error=error,prediction_error=prediction,
                        disturbance_estimate=estimated_d,observer_residual_truth=truth,actor_norm=na,critic_norm=nc,
                        tracking_error=[v[k]-vc[k] for k in range(3)])
        return result

    def step(self,t,state,vc,agent,dt):
        # Explicit midpoint: one common integrator, no position/velocity clipping.
        k1=self.rhs(t,state,vc,agent)
        mid=[x+dt*.5*y for x,y in zip(state,k1)]
        k2=self.rhs(t+dt*.5,mid,vc,agent)
        return [x+dt*y for x,y in zip(state,k2)]
