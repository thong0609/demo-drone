/* Strictly convex QP: min .5 x'Hx + f'x, Ax<=b.
 * Hildreth dual coordinate descent; no ADMM or post-hoc command clipping.
 * Hinv is supplied by a checked factorization. Returns normalized primal
 * violation and complementarity separately; feasible/inexact is not optimal.
 */
#include <math.h>
#include <stdlib.h>
int solve_qp(int n,int m,const double *inv,const double *f,const double *A,
             const double *b,int max_sweeps,double tol,double *x,double *stats){
    double *directions=calloc((size_t)m*n,sizeof(double));
    double *diagonal=calloc(m,sizeof(double));
    double *lambda=calloc(m,sizeof(double));
    double *row_norm=calloc(m,sizeof(double));
    if(!directions||!diagonal||!lambda||!row_norm){free(directions);free(diagonal);free(lambda);free(row_norm);return -1;}
    for(int i=0;i<n;i++){x[i]=0;for(int j=0;j<n;j++)x[i]-=inv[i*n+j]*f[j];}
    for(int r=0;r<m;r++){
        for(int i=0;i<n;i++)row_norm[r]+=A[r*n+i]*A[r*n+i];
        row_norm[r]=fmax(1e-12,sqrt(row_norm[r]));
    }
    int iterations=0;double violation=0,comp=0;
    for(int sweep=0;sweep<max_sweeps;sweep++){
        for(int r=0;r<m;r++){
            double residual=-b[r];for(int i=0;i<n;i++)residual+=A[r*n+i]*x[i];
            if(residual<=0 && lambda[r]==0)continue;
            /* Lazy inverse-row product: inactive inequalities need no solve. */
            if(diagonal[r]==0){
                for(int i=0;i<n;i++)for(int j=0;j<n;j++)
                    if(inv[i*n+j]!=0 && A[r*n+j]!=0)
                        directions[r*n+i]+=inv[i*n+j]*A[r*n+j];
                for(int i=0;i<n;i++)diagonal[r]+=A[r*n+i]*directions[r*n+i];
                if(diagonal[r]<1e-16)diagonal[r]=1e-16;
            }
            double next=fmax(0,lambda[r]+residual/diagonal[r]);
            double change=next-lambda[r];lambda[r]=next;
            if(change!=0)for(int i=0;i<n;i++)x[i]-=change*directions[r*n+i];
        }
        violation=0;comp=0;
        for(int r=0;r<m;r++){
            double residual=-b[r];for(int i=0;i<n;i++)residual+=A[r*n+i]*x[i];
            violation=fmax(violation,residual/row_norm[r]);comp=fmax(comp,fabs(lambda[r]*residual));
        }
        iterations=sweep+1;
        if(violation<=tol && comp<=tol)break;
    }
    stats[0]=iterations;stats[1]=violation;stats[2]=comp;
    free(directions);free(diagonal);free(lambda);free(row_norm);
    return violation<=tol ? (comp<=tol?0:1):2;
}

/* Exact closest projection onto exported piecewise-linear tube. */
void project_tube(int n,const double *p,const double *r,const double *query,double *out){
    double best=1e300;
    for(int i=0;i<n-1;i++){
        double dx=p[3*i+3]-p[3*i],dy=p[3*i+4]-p[3*i+1],dz=p[3*i+5]-p[3*i+2];
        double l2=dx*dx+dy*dy+dz*dz,l=sqrt(l2);
        double t=((query[0]-p[3*i])*dx+(query[1]-p[3*i+1])*dy+(query[2]-p[3*i+2])*dz)/l2;
        t=fmax(0,fmin(1,t));
        double x=p[3*i]+t*dx,y=p[3*i+1]+t*dy,z=p[3*i+2]+t*dz;
        double distance=(query[0]-x)*(query[0]-x)+(query[1]-y)*(query[1]-y)+(query[2]-z)*(query[2]-z);
        if(distance<best){
            best=distance;double radius=r[i]*(1-t)+r[i+1]*t;
            out[0]=i;out[1]=t;out[2]=radius;out[3]=x;out[4]=y;out[5]=z;
            out[6]=dx/l;out[7]=dy/l;out[8]=dz/l;out[9]=(r[i+1]-r[i])/l;out[10]=radius-sqrt(distance);
        }
    }
}

double min_pair_distance(int N,int K,const double *own_start,const double *own,
                        const double *other_start,const double *others){
    double minimum=1e300;
    for(int j=0;j<K;j++)for(int k=0;k<N;k++){
        const double *a=k?own+3*(k-1):own_start;
        const double *b=own+3*k;
        const double *c=k?others+3*(j*N+k-1):other_start+3*j;
        const double *d=others+3*(j*N+k);
        double rel[3],delta[3],den=0,num=0;
        for(int axis=0;axis<3;axis++){rel[axis]=a[axis]-c[axis];delta[axis]=b[axis]-a[axis]-d[axis]+c[axis];den+=delta[axis]*delta[axis];num+=rel[axis]*delta[axis];}
        double t=den>1e-12?fmax(0,fmin(1,-num/den)):0,dist=0;
        for(int axis=0;axis<3;axis++){double v=rel[axis]+t*delta[axis];dist+=v*v;}
        minimum=fmin(minimum,sqrt(dist));
    }
    return minimum;
}
