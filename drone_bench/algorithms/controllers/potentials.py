"""Analytic potentials and derivatives, Eqs. 25–33 (metres)."""
import math


def phi(x, d1, d2):
    a=math.pi/(2*(d2-d1))
    if x<d1: return 1-a*(x-d1), -a
    if x<d2: return 1-math.sin(a*(x-d1)), -a*math.cos(a*(x-d1))
    return 0.0,0.0


def repulsive(x, safety, activation, gain, delta):
    if x <= 0: raise ValueError('Nonpositive clearance in repulsive potential')
    value, derivative=phi(x,safety,activation)
    psi=2/math.pi*math.atan(x/safety)
    denominator=(1+delta)*x-safety*psi
    denominator_derivative=1+delta-2/math.pi/(1+(x/safety)**2)
    return gain*value/denominator, gain*(derivative*denominator-value*denominator_derivative)/denominator**2


def cohesive(x, ls, lm, gain):
    if x<ls: return 0.0,0.0
    a=math.pi/(2*(lm-ls))
    if x<lm: mu,derivative=1-math.cos(a*(x-ls)),a*math.sin(a*(x-ls))
    else: mu,derivative=1+a*(x-lm),a
    nu=1-2/math.pi*math.atan(x/ls)
    denominator=ls*nu; dd=-2/math.pi/(1+(x/ls)**2)
    return gain*mu/denominator,gain*(derivative*denominator-mu*dd)/denominator**2
