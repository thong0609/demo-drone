"""Small native QP backend, compiled locally; convex norm bounds via inner polytope."""
import ctypes
import hashlib
import itertools
import math
from pathlib import Path
import subprocess
from ....common.linear import factor, solve


def polytope():
    """26 supporting normals scaled so every polytope vertex lies in unit ball."""
    directions=[]
    for vector in itertools.product((-1,0,1),repeat=3):
        length=math.sqrt(sum(x*x for x in vector))
        if length:directions.append([x/length for x in vector])
    maximum=1.0
    for a,b,c in itertools.combinations(directions,3):
        determinant=a[0]*(b[1]*c[2]-b[2]*c[1])-a[1]*(b[0]*c[2]-b[2]*c[0])+a[2]*(b[0]*c[1]-b[1]*c[0])
        if abs(determinant)<1e-9:continue
        def cross(u,v):return [u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]]
        terms=[cross(b,c),cross(c,a),cross(a,b)]
        vertex=[sum(v[k] for v in terms)/determinant for k in range(3)]
        if all(sum(x*y for x,y in zip(n,vertex))<=1+1e-8 for n in directions):
            maximum=max(maximum,math.sqrt(sum(x*x for x in vertex)))
    return directions,1/maximum


class QPSolver:
    def __init__(self,hessian):
        self.n=len(hessian)
        lu=factor(hessian)
        columns=[solve(lu,[float(i==j) for i in range(self.n)]) for j in range(self.n)]
        self.inverse=[columns[j][i] for i in range(self.n) for j in range(self.n)]
        source=Path(__file__).with_name('qp.c')
        digest=hashlib.sha256(source.read_bytes()).hexdigest()[:16]
        target=Path('.cache/dmpc')/f'qp_{digest}.so';target.parent.mkdir(parents=True,exist_ok=True)
        if not target.exists():
            subprocess.run(['gcc','-O3','-std=c99','-shared','-fPIC',str(source),'-lm','-o',str(target)],check=True,capture_output=True)
        self.library=ctypes.CDLL(str(target.resolve()))
        ptr=ctypes.POINTER(ctypes.c_double)
        self.library.solve_qp.argtypes=[ctypes.c_int,ctypes.c_int,ptr,ptr,ptr,ptr,ctypes.c_int,ctypes.c_double,ptr,ptr]
        self.library.solve_qp.restype=ctypes.c_int
        self.inverse_array=(ctypes.c_double*(self.n*self.n))(*self.inverse)

    def solve(self,gradient,rows,bounds,max_sweeps=180,tolerance=1e-4):
        # Native code reports row-normalized residuals. Keeping raw coefficients
        # here avoids a Python normalization pass over every horizon constraint.
        flat=[value for row in rows for value in row]
        limits=list(bounds)
        n=self.n;m=len(limits)
        f=(ctypes.c_double*n)(*gradient);A=(ctypes.c_double*(m*n))(*flat);b=(ctypes.c_double*m)(*limits)
        x=(ctypes.c_double*n)();stats=(ctypes.c_double*3)()
        result=self.library.solve_qp(n,m,self.inverse_array,f,A,b,max_sweeps,tolerance,x,stats)
        if result<0:raise RuntimeError('QP allocation failed')
        answer=list(x)
        if not all(math.isfinite(v) for v in answer):return None,dict(status='nonfinite')
        return answer,dict(status=['optimal','feasible_inexact','residual_exceeded'][result],
                           sweeps=int(stats[0]),violation=stats[1],complementarity=stats[2],constraints=m)
