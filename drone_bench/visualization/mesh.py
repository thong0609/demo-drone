"""Display-only tube mesh; never used for collision queries."""
import math
from ..common.vectors import sub, unit, cross, add, mul

def mesh(tube):
    rings=[]; previous=None
    for i,p in enumerate(tube.points):
        tangent=unit(sub(tube.points[min(i+1,len(tube.points)-1)],tube.points[max(i-1,0)]))
        if previous is None:
            normal=unit(cross(tangent,[0,0,1] if abs(tangent[2])<0.9 else [0,1,0]))
        else:
            normal=unit(sub(previous,mul(tangent,sum(a*b for a,b in zip(previous,tangent)))))
        previous=normal; binormal=cross(tangent,normal)
        rings.append([add(p,mul(add(mul(normal,math.cos(j*2*math.pi/24)),mul(binormal,math.sin(j*2*math.pi/24))),tube.radii[i])) for j in range(24)])
    return rings


