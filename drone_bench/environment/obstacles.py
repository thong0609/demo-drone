"""Analytical distances to obstacle primitives."""
from ..common.vectors import norm

def box_sdf(p, lo, hi):
    q = [abs(x-(a+b)/2)-(b-a)/2 for x, a, b in zip(p, lo, hi)]
    return norm([max(x, 0) for x in q]) + min(max(q), 0)


