"""Small dense LU solver with partial pivoting, no external dependencies."""

def factor(matrix):
    a=[row[:] for row in matrix]; n=len(a); swaps=[]
    for k in range(n):
        pivot=max(range(k,n),key=lambda i: abs(a[i][k]))
        if abs(a[pivot][k]) < 1e-13: raise ValueError('Singular QP system')
        a[k],a[pivot]=a[pivot],a[k]; swaps.append(pivot)
        for i in range(k+1,n):
            v=a[i][k]/a[k][k]; a[i][k]=v
            for j in range(k+1,n): a[i][j]-=v*a[k][j]
    return a,swaps


def solve(factors, rhs):
    a,swaps=factors; b=list(rhs); n=len(b)
    for k,p in enumerate(swaps): b[k],b[p]=b[p],b[k]
    for i in range(n): b[i]-=sum(a[i][j]*b[j] for j in range(i))
    for i in range(n-1,-1,-1): b[i]=(b[i]-sum(a[i][j]*b[j] for j in range(i+1,n)))/a[i][i]
    return b
