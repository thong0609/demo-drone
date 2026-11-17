"""Sampled shape diagnostics, separate from continuous collision certificates."""
import math
from ...common.vectors import sub, norm, dot, mul, add, cross
from ...tubes.polynomial import evaluate, derivative


def shape_metrics(tube, path=None):
    length=sum(norm(sub(b,a)) for a,b in zip(tube.points,tube.points[1:]))
    path=path or [tube.points[0],tube.points[-1]]
    raw=sum(norm(sub(b,a)) for a,b in zip(path,path[1:]))
    direction=sub(tube.points[-1],tube.points[0]);direction=mul(direction,1/norm(direction))
    backward=sum(max(0.,-dot(sub(b,a),direction)) for a,b in zip(tube.points,tube.points[1:]))
    curvature=[]
    if hasattr(tube,'polynomials'):
        for span in tube.polynomials:
            d=[derivative(c) for c in span[:3]];dd=[derivative(c) for c in d]
            for j in range(64):
                t=(j+.5)/64;v=[evaluate(c,t) for c in d];a=[evaluate(c,t) for c in dd]
                if norm(v)>1e-6:curvature.append(norm(cross(v,a))/norm(v)**3)
    return dict(length=length,waypoint_length=raw,length_inflation=length/raw,
                backward_distance_along_start_goal=backward,
                sampled_max_curvature=max(curvature,default=None),min_radius=min(tube.radii),
                scope='sampled shape metrics; curvature is not a certified maximum; backward is diagnostic only')


def shape_check(tube, cfg, path):
    metrics=shape_metrics(tube,path)
    limit=cfg.get('max_length_inflation')
    if limit is not None and metrics['length_inflation']>limit:
        raise ValueError('Tube length inflation exceeds quality threshold')
    return metrics


def displacement(before,after,count=65):
    """Compare corresponding normalized sampled arc lengths, not nearest distance."""
    def samples(tube):
        distances=[0.]
        for a,b in zip(tube.points,tube.points[1:]):distances.append(distances[-1]+norm(sub(b,a)))
        result=[];k=0
        for j in range(count):
            target=distances[-1]*j/(count-1)
            while k<len(distances)-2 and distances[k+1]<target:k+=1
            f=(target-distances[k])/(distances[k+1]-distances[k])
            result.append(add(tube.points[k],mul(sub(tube.points[k+1],tube.points[k]),f)))
        return result
    values=[norm(sub(a,b)) for a,b in zip(samples(before),samples(after))]
    return dict(mean=sum(values)/len(values),maximum=max(values),samples=count)
