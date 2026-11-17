"""Deterministic, validated 3×3×3 swarm start configuration."""
from ..common.vectors import add, sub, mul, unit, cross, norm


def initial_positions(tube, cfg, safety_radius):
    lengths=[norm(sub(b,a)) for a,b in zip(tube.points,tube.points[1:])]
    result=[]
    for station in cfg['initial_stations']:
        remaining=station
        for i,length in enumerate(lengths):
            if remaining<=length: break
            remaining-=length
        else: raise ValueError('Initial station exceeds tube length')
        fraction=remaining/length
        center=add(tube.points[i],mul(sub(tube.points[i+1],tube.points[i]),fraction))
        tangent=unit(sub(tube.points[i+1],tube.points[i]))
        normal=unit(cross(tangent,[0,0,1])); binormal=cross(tangent,normal)
        for row in (-1,0,1):
            for column in (-1,0,1):
                result.append(add(center,add(mul(normal,row*cfg['grid_spacing']),mul(binormal,column*cfg['grid_spacing']))))
    if len(result)!=27: raise ValueError('V-B initial swarm must contain 27 UAVs')
    if min(norm(sub(a,b)) for i,a in enumerate(result) for b in result[i+1:])<2*safety_radius:
        raise ValueError('Initial swarm safety regions overlap')
    if min(tube.nearest_boundary(p)['distance'] for p in result)<safety_radius or min(tube.project(p)['radial_clearance'] for p in result)<0:
        raise ValueError('Initial swarm violates tube clearance')
    return result
