"""Continuous polynomial tube, Eqs. 10–13, with numerical surface queries.

The sampled points remain a rendering/export aid. Queries evaluate the Bernstein
polynomials, including their derivatives. All polynomial spans are searched;
the minimum is refined in continuous parameter space, not on a triangle mesh.
End disks are entry/exit portals, so boundary queries concern the lateral wall.
"""
import math
from .model import Tube
from ..common.vectors import sub, dot, norm, mul, add, cross, unit


def power_coefficients(control):
    n=len(control)-1
    return [math.comb(n,k)*sum((-1)**(k-j)*math.comb(k,j)*control[j]
                              for j in range(k+1)) for k in range(n+1)]


def evaluate(coefficients,t):
    value=0.0
    for c in reversed(coefficients): value=value*t+c
    return value


def derivative(coefficients):
    return [j*x for j,x in enumerate(coefficients)][1:]


def minimize(function,a,b,tolerance=1e-9):
    ratio=(math.sqrt(5)-1)/2
    x=b-ratio*(b-a);y=a+ratio*(b-a);fx=function(x);fy=function(y)
    while b-a>tolerance:
        if fx<fy:
            b,y,fy=y,x,fx;x=b-ratio*(b-a);fx=function(x)
        else:
            a,x,fx=x,y,fy;y=a+ratio*(b-a);fy=function(y)
    candidates=[(fx,x),(fy,y),(function(a),a),(function(b),b)]
    return min(candidates)


class PolynomialTube(Tube):
    def __init__(self,points,radii,curve):
        super().__init__(points,radii)
        self.curve=curve
        axes=curve['centerline_coefficients'];radius=curve['radius_coefficients']
        self.spans=len(radius)
        if len(axes)!=3 or any(len(axis)!=self.spans for axis in axes):
            raise ValueError('Invalid polynomial tube dimensions')
        if len(curve['durations'])!=self.spans or any(t<=0 for t in curve['durations']):
            raise ValueError('Invalid polynomial tube durations')
        if any(len(c)!=8 or any(not math.isfinite(v) for v in c) for axis in axes for c in axis):
            raise ValueError('Expected finite degree-7 Bernstein centerline controls')
        if any(len(c)!=8 or any(not math.isfinite(v) or v<=0 for v in c) for c in radius):
            raise ValueError('Expected positive finite degree-7 Bernstein radius controls')
        self.polynomials=[[power_coefficients(axis[s]) for axis in axes]+[power_coefficients(radius[s])]
                          for s in range(self.spans)]
        self.derivatives=[[derivative(c) for c in span] for span in self.polynomials]
        self.resolution=64
        self.grid=[[self.sample(s,j/self.resolution) for j in range(self.resolution+1)]
                   for s in range(self.spans)]
        self.arc_offsets=[0.0];self.arc_grid=[]
        for s in range(self.spans):
            values=[0.0]
            for j in range(self.resolution):
                values.append(values[-1]+self.arc_interval(s,j/self.resolution,(j+1)/self.resolution))
            self.arc_grid.append(values);self.arc_offsets.append(self.arc_offsets[-1]+values[-1])
        self.representation='continuous Bernstein polynomial; normal disks'
        parameters=curve.get('sample_parameters')
        if parameters is not None:
            if len(parameters)!=len(points):raise ValueError('Polynomial sample metadata length mismatch')
            for (s,t),point,r in zip(parameters,points,radii):
                if not 0<=s<self.spans or not 0<=t<=1:raise ValueError('Invalid polynomial sample parameter')
                center,radius,_,_=self.sample(s,t)
                if norm(sub(center,point))>1e-5 or abs(radius-r)>1e-5:
                    raise ValueError('Polynomial coefficients and rendering samples disagree')

    def sample(self,s,t):
        coeff=self.polynomials[s];der=self.derivatives[s]
        center=[evaluate(c,t) for c in coeff[:3]];r=evaluate(coeff[3],t)
        velocity=[evaluate(c,t) for c in der[:3]];dr=evaluate(der[3],t)
        speed=norm(velocity)
        if speed<1e-8:
            # Eq. 17 permits zero endpoint speed/acceleration. Use the one-sided
            # tangent limit rather than a chord or an undefined unit vector.
            higher=der
            for order in range(2,8):
                higher=[derivative(c) for c in higher]
                sign=(-1)**(order-1) if t>.5 else 1
                velocity=[sign*evaluate(c,t) for c in higher[:3]]
                speed=norm(velocity)
                if speed>=1e-8:
                    dr=sign*evaluate(higher[3],t);break
        if speed<1e-8 or r<=0: raise ValueError('Degenerate polynomial tube')
        return center,r,mul(velocity,1/speed),dr/speed

    @staticmethod
    def ring_distance_squared(p,sample):
        center,r,tangent,_=sample
        x=p[0]-center[0];y=p[1]-center[1];z=p[2]-center[2]
        h=x*tangent[0]+y*tangent[1]+z*tangent[2]
        rho=math.sqrt(max(0,x*x+y*y+z*z-h*h))
        return h*h+(rho-r)**2

    @staticmethod
    def ring(p,sample):
        center,r,tangent,_=sample
        delta=sub(p,center);axial=dot(delta,tangent)
        radial=sub(delta,mul(tangent,axial));rho=norm(radial)
        if rho>1e-12: direction=mul(radial,1/rho)
        else: direction=unit(cross(tangent,[0,0,1] if abs(tangent[2])<.9 else [0,1,0]))
        point=add(center,mul(direction,r))
        return axial*axial+(rho-r)**2,point

    def search(self,p,surface=False):
        """Multistart continuous minimization on every span (numerical tolerance).

        Bracket every grid-local minimum, including both end neighborhoods. This
        avoids a stale agent cache selecting the wrong distant part of the tube.
        It is a numerical search, not a certified global-distance bound.
        """
        best=None
        for s,grid in enumerate(self.grid):
            if surface:
                values=[self.ring_distance_squared(p,q) for q in grid]
                function=lambda t:self.ring_distance_squared(p,self.sample(s,t))
            else:
                values=[sum((a-b)**2 for a,b in zip(p,q[0])) for q in grid]
                function=lambda t:sum((p[k]-evaluate(c,t))**2 for k,c in enumerate(self.polynomials[s][:3]))
            n=self.resolution
            candidates=[(values[0],0.0),(values[-1],1.0)]
            for j in range(n+1):
                if (j==0 or values[j]<=values[j-1]) and (j==n or values[j]<=values[j+1]):
                    candidates.append(minimize(function,max(0,j-1)/n,min(n,j+1)/n))
            value,t=min(candidates)
            if best is None or value<best[0]:best=(value,s,t)
        return best

    def project(self,p):
        _,s,t=self.search(p)
        center,r,tangent,slope=self.sample(s,t);offset=sub(p,center);distance=norm(offset)
        return dict(segment=s,fraction=t,point=center,offset=offset,distance=distance,
                    radius=r,tangent=tangent,radius_slope=slope,
                    radial_clearance=r-distance,arc=self.arc_at(s,t))

    def arc_interval(self,s,a,b):
        def speed(t):return math.sqrt(sum(evaluate(c,t)**2 for c in self.derivatives[s][:3]))
        return (b-a)/6*(speed(a)+4*speed((a+b)/2)+speed(b))

    def arc_at(self,s,t):
        j=min(self.resolution-1,int(t*self.resolution))
        return self.arc_offsets[s]+self.arc_grid[s][j]+self.arc_interval(s,j/self.resolution,t)

    def nearest_boundary(self,p):
        _,s,t=self.search(p,True)
        _,point=self.ring(p,self.sample(s,t));delta=sub(p,point)
        return dict(point=point,distance=norm(delta),inward=unit(delta) if norm(delta)>1e-12 else [0,0,0],
                    segment=s,fraction=t)

    def guidance_length(self,location):
        # Simpson integration of the derivative of Eq. 22, in the polynomial
        # parameter; no substitution by polyline chords.
        offset=mul(location['offset'],1/location['radius']);total=0.0
        for s in range(location['segment'],self.spans):
            start=location['fraction'] if s==location['segment'] else 0.0
            n=64;h=(1-start)/n
            for j in range(n+1):
                t=start+j*h;d=self.derivatives[s]
                velocity=add([evaluate(c,t) for c in d[:3]],mul(offset,evaluate(d[3],t)))
                total+=norm(velocity)*h/3*(1 if j in (0,n) else 4 if j%2 else 2)
        return total

    def validate(self,env,margin):
        # Certificate for the continuous curve, not just its sampled chords:
        # every Bernstein center is in the control-point convex hull; the SDF
        # is 1-Lipschitz. Subdivide until each enclosure fits in free space.
        def split(control):
            levels=[list(control)]
            while len(levels[-1])>1:
                levels.append([(a+b)/2 for a,b in zip(levels[-1],levels[-1][1:])])
            return [row[0] for row in levels],[row[-1] for row in reversed(levels)]
        minimum=float('inf')
        def certify(axes,radius,depth=0):
            nonlocal minimum
            center=[sum(c)/len(c) for c in axes]
            extent=math.sqrt(sum(max(abs(v-center[k]) for v in c)**2 for k,c in enumerate(axes)))
            lower=env.clearance(center)-extent-max(radius)
            if lower>=margin-1e-8:
                minimum=min(minimum,lower);return
            if depth>=18:raise ValueError('Continuous polynomial tube clearance certificate failed')
            halves=[split(c) for c in axes];rl,rr=split(radius)
            certify([c[0] for c in halves],rl,depth+1)
            certify([c[1] for c in halves],rr,depth+1)
        for s in range(self.spans):
            certify([axis[s] for axis in self.curve['centerline_coefficients']],self.curve['radius_coefficients'][s])
        return dict(certified_obstacle_and_world_margin=minimum,
                    length=sum(norm(sub(b,a)) for a,b in zip(self.points,self.points[1:])),
                    min_radius=min(self.radii),max_radius=max(self.radii),
                    certificate='Bernstein convex-hull subdivision of continuous swept balls')
