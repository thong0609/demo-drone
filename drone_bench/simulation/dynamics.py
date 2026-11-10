"""Translational model Eq. 1 with explicit, reproducible bounded perturbations."""
import math


def known_dynamics(v,cfg):
    return [-cfg['drag'][k]*v[k]/cfg['mass']-(cfg['gravity'] if k==2 else 0) for k in range(3)]


def perturbations(t,p,v,agent,cfg):
    model=cfg.get('uncertainty_model','position_velocity')
    if model not in ('velocity','position_velocity'):raise ValueError('Unknown uncertainty model')
    uncertainty=[cfg['uncertainty_amplitude']*math.sin(v[k]/4+(p[k]/80 if model=='position_velocity' else 0)) for k in range(3)]
    disturbance=[cfg['disturbance_amplitude']*math.sin((.45+.1*k)*t+.37*agent+k) for k in range(3)]
    return uncertainty,disturbance
