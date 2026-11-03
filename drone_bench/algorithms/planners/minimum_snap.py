"""Eqs. 17–21: degree-7 Bernstein minimum-snap QP, corridor constraints.

Equality constraints are exact; conservative convex-hull corridor inequalities
are solved with an active set. Units of the polynomial parameter are normalized;
segment durations account for derivative scaling.
"""
import math
from ...common.linear import factor, solve


def bernstein(n, t):
    return [math.comb(n,i)*t**i*(1-t)**(n-i) for i in range(n+1)]


def basis_derivative(t, order=0):
    if not order: return bernstein(7,t)
    base=bernstein(7-order,t); scale=math.factorial(7)/math.factorial(7-order)
    result=[0.0]*8
    for i,b in enumerate(base):
        for k in range(order+1): result[i+k]+=scale*b*(-1)**(order-k)*math.comb(order,k)
    return result


def eval_curve(coefficients, segment, t, order=0):
    b=basis_derivative(t,order)
    return [sum(x*y for x,y in zip(c[segment],b)) for c in coefficients]


def minimum_snap(values, durations, corridor, gamma, endpoint_speeds=(0,0)):
    """Return Bernstein control points and independently checked QP residuals."""
    segments=len(durations); n=8*segments
    h=[[0.0]*n for _ in range(n)]; f=[0.0]*n
    # Exact Bernstein product integrals for snap and tracking objective.
    for s,T in enumerate(durations):
        d=[[0.0]*8 for _ in range(4)]
        for i in range(4):
            for k in range(5): d[i][i+k]=840*(-1)**(4-k)*math.comb(4,k)
        for i in range(8):
            for j in range(8):
                snap=sum(d[a][i]*d[b][j]*math.comb(3,a)*math.comb(3,b)/(7*math.comb(6,a+b))
                         for a in range(4) for b in range(4))/T**7
                mass=math.comb(7,i)*math.comb(7,j)/(15*math.comb(14,i+j))
                h[8*s+i][8*s+j]=2*(snap+gamma*T*mass)
                target=values[s]+(values[s+1]-values[s])*j/7
                f[8*s+i]-=2*gamma*T*mass*target
    scale=max(max(abs(x) for x in row) for row in h)
    h=[[x/scale for x in row] for row in h]; f=[x/scale for x in f]
    for i in range(n): h[i][i]+=1e-11
    A=[]; b=[]
    def row(s,t,order=0):
        r=[0.0]*n
        r[s*8:s*8+8]=[x/durations[s]**order for x in basis_derivative(t,order)]
        return r
    for s in range(segments):
        A.extend([row(s,0),row(s,1)]); b.extend(values[s:s+2])
    for s in range(segments-1):
        for order in (1,2):
            A.append([x-y for x,y in zip(row(s,1,order),row(s+1,0,order))]); b.append(0)
    for order in (1,2):
        A.extend([row(0,0,order),row(segments-1,1,order)])
        b.extend(endpoint_speeds if order==1 else (0,0))
    G=[]; limits=[]
    # Convex hull corridor: bound Bernstein controls relative to linear controls.
    # This enforces the entire continuous curve's coordinate deviation, not samples.
    for s in range(segments):
        for j in range(1,7):
            r=[0.0]*n; r[s*8+j]=1
            target=values[s]+(values[s+1]-values[s])*j/7
            for sign in (1,-1):
                G.append([sign*x for x in r]); limits.append(sign*target+corridor)
    # Use a phase-independent equality solution first. Inactive corridor is common.
    def equality_solve(rows, rhs, gradient):
        m=len(rows)
        kkt=[h[i][:]+[r[i] for r in rows] for i in range(n)]
        kkt += [r[:]+[0.0]*m for r in rows]
        return solve(factor(kkt),[-v for v in gradient]+list(rhs))
    answer=equality_solve(A,b,f); x=answer[:n]
    active=[]
    # Dual active-set constraint generation with removal of negative multipliers.
    # Fail explicitly on nonconvergence instead of accepting an infeasible curve.
    for iteration in range(250):
        violations=[sum(v*w for v,w in zip(r,x))-lim for r,lim in zip(G,limits)]
        worst=max(range(len(G)),key=violations.__getitem__)
        if violations[worst] <= 1e-6:
            multipliers=answer[n+len(A):]
            if not multipliers or min(multipliers)>=-1e-7: break
            active.pop(min(range(len(active)),key=lambda i:multipliers[i]))
        else:
            if worst in active: raise RuntimeError('QP active set stalled')
            active.append(worst)
        answer=equality_solve(A+[G[i] for i in active],b+[limits[i] for i in active],f)
        x=answer[:n]
    else: raise RuntimeError('Minimum snap QP did not converge')
    eq=max(abs(sum(v*w for v,w in zip(r,x))-target) for r,target in zip(A,b))
    violation=max(0,max(sum(v*w for v,w in zip(r,x))-lim for r,lim in zip(G,limits)))
    if eq>1e-5 or violation>1e-5: raise RuntimeError('QP residual check failed')
    return [x[8*s:8*s+8] for s in range(segments)], {'equality_residual':eq,'corridor_violation':violation,'active_constraints':len(active),'iterations':iteration+1}
