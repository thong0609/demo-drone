"""Algorithm-independent tube representation and validation."""
from ..common.vectors import add, sub, mul, dot, norm, cross, unit

class Tube:
    """Piecewise-linear centerline with linearly interpolated radius.

    Normal cross-sections define the rendered tube. The swept balls around every
    segment conservatively enclose those disks and are used for collision checks.
    """
    def __init__(self, points, radii):
        if len(points) < 2 or len(points) != len(radii): raise ValueError('Invalid tube arrays')
        if any(r <= 0 for r in radii): raise ValueError('Radius must be positive')
        if any(norm(sub(b,a)) < 1e-10 for a,b in zip(points,points[1:])): raise ValueError('Duplicate centerline samples')
        self.points, self.radii = points, radii

    def project(self, p):
        """Nearest centerline projection; local radial clearance (not global SDF)."""
        best = None
        for i, (a,b) in enumerate(zip(self.points, self.points[1:])):
            d = sub(b,a)
            t = max(0, min(1, dot(sub(p,a),d)/dot(d,d)))
            q = add(a,mul(d,t)); distance = norm(sub(p,q))
            if best is None or distance < best['distance']:
                r = self.radii[i]*(1-t)+self.radii[i+1]*t
                best = dict(segment=i, fraction=t, point=q, distance=distance,
                            radius=r, radial_clearance=r-distance)
        return best

    def nearest_boundary(self,p):
        """Nearest lateral frustum surface for a sampled tube.

        Entry/exit disks are portals. For a straight, linearly tapered tube the
        minimizer below is analytic, including its axial displacement.
        """
        best=None
        for i,(a,b) in enumerate(zip(self.points,self.points[1:])):
            d=sub(b,a);length=norm(d);tangent=mul(d,1/length)
            delta=sub(p,a);axial=dot(delta,tangent)
            radial=sub(delta,mul(tangent,axial));rho=norm(radial)
            direction=mul(radial,1/rho) if rho>1e-12 else unit(cross(tangent,[0,0,1] if abs(tangent[2])<.9 else [0,1,0]))
            dr=self.radii[i+1]-self.radii[i]
            t=max(0,min(1,(length*axial+dr*(rho-self.radii[i]))/(length*length+dr*dr)))
            point=add(add(a,mul(d,t)),mul(direction,self.radii[i]+dr*t))
            difference=sub(p,point);distance=norm(difference)
            if best is None or distance<best['distance']:
                best=dict(point=point,distance=distance,inward=mul(difference,1/distance) if distance>1e-12 else [0,0,0],segment=i,fraction=t)
        return best

    def validate(self, env, margin):
        # Every point of segment is within length/2 of one endpoint.
        # Subtract maximum interpolated radius to enclose the whole swept tube.
        bounds = [min(env.clearance(a), env.clearance(b))-norm(sub(b,a))/2-max(ra,rb)
                  for a,b,ra,rb in zip(self.points,self.points[1:],self.radii,self.radii[1:])]
        lower = min(bounds)
        if lower < margin-1e-8: raise ValueError(f'Tube clearance certificate failed: {lower:.6f} < {margin}')
        return {'certified_obstacle_and_world_margin': lower,
                'length': sum(norm(sub(b,a)) for a,b in zip(self.points,self.points[1:])),
                'min_radius': min(self.radii), 'max_radius': max(self.radii)}


