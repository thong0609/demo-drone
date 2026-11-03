"""Independent checks of saved results and native QP against known solutions.

Run from repository root: python3 -m experiments.audit_dmpc
Does not modify controller, plant, config, or experiment outputs.
"""
import csv
import hashlib
import itertools
import json
import math
import random
from pathlib import Path
from drone_bench.algorithms.controllers.dmpc.optimizer import QPSolver
from drone_bench.common.linear import factor,solve
from drone_bench.io.benchmark import load_benchmark,write_json
from drone_bench.common.vectors import dot,sub,norm,unit
from drone_bench.tubes.model import Tube


def numerical_checks():
    rng=random.Random(482);small_errors=[]
    for case in range(40):
        n=2 if case<20 else 3
        R=[[rng.uniform(-1,1) for _ in range(n)] for _ in range(n)]
        H=[[sum(R[k][i]*R[k][j] for k in range(n))+(1 if i==j else 0) for j in range(n)] for i in range(n)]
        f=[rng.uniform(-5,5) for _ in range(n)];A=[];b=[]
        for i in range(n):
            row=[float(i==j) for j in range(n)];A.extend([row,[-x for x in row]]);b.extend([1,1])
        for _ in range(3):A.append([rng.uniform(-1,1) for _ in range(n)]);b.append(rng.uniform(.2,1))
        best=None
        for count in range(n+1):
            for active in itertools.combinations(range(len(A)),count):
                rows=[A[i] for i in active]
                kkt=[H[i]+[r[i] for r in rows] for i in range(n)]+[r+[0.0]*count for r in rows]
                try:solution=solve(factor(kkt),[-x for x in f]+[b[i] for i in active])
                except ValueError:continue
                x=solution[:n]
                if min(solution[n:],default=0)<-1e-8 or max(sum(u*v for u,v in zip(a,x))-bound for a,bound in zip(A,b))>1e-8:continue
                objective=.5*sum(x[i]*H[i][j]*x[j] for i in range(n) for j in range(n))+sum(u*v for u,v in zip(f,x))
                if best is None or objective<best[0]:best=(objective,x)
        answer,_=QPSolver(H).solve(f,A,b,max_sweeps=5000,tolerance=1e-9)
        small_errors.append(norm(sub(answer,best[1])))
    rng=random.Random(984);large_errors=[]
    for _ in range(12):
        n=31;m=120
        R=[[rng.uniform(-.2,.2) for _ in range(n)] for _ in range(n)]
        H=[[sum(R[k][i]*R[k][j] for k in range(n))+(1 if i==j else 0) for j in range(n)] for i in range(n)];H[-1][-1]+=399
        expected=[rng.uniform(-1,1) for _ in range(n)];A=[[rng.uniform(-1,1) for _ in range(n)] for _ in range(m)]
        lam=[rng.uniform(.2,2) if j<10 else 0 for j in range(m)]
        b=[dot(row,expected)+(0 if j<10 else rng.uniform(.2,2)) for j,row in enumerate(A)]
        f=[-dot(H[i],expected)-sum(A[j][i]*lam[j] for j in range(m)) for i in range(n)]
        answer,_=QPSolver(H).solve(f,A,b,max_sweeps=5000,tolerance=1e-9)
        large_errors.append(norm(sub(answer,expected)))
    return dict(active_set_enumeration=dict(cases=40,max_solution_error=max(small_errors),passed=max(small_errors)<1e-5),
                known_KKT_31_variables=dict(cases=12,max_solution_error=max(large_errors),passed=max(large_errors)<1e-5))


def main():
    out=Path('artifacts/dmpc_audit');out.mkdir(exist_ok=True)
    branch=Path('artifacts/outer_comparison/dmpc')
    summary=json.load(open(branch/'summary.json'));trajectory=json.load(open(branch/'trajectory.json'))
    full=list(csv.DictReader(open(branch/'metrics.csv')))
    short=list(csv.DictReader(open('artifacts/dmpc_validation/uav_27/metrics.csv')))
    extrema={}
    for key,fn in [('min_pair_distance',min),('max_pair_distance',max),('min_boundary_clearance',min),('max_speed',max),('max_input',max)]:
        value=fn(float(r[key]) for r in full if r[key])
        extrema[key]=dict(recomputed=value,reported=summary[key],matches=abs(value-summary[key])<1e-8)
    frame_pairs=[];frame_speeds=[]
    for frame in trajectory['frames']:
        for k,i in enumerate(frame['active']):
            frame_speeds.append(norm(frame['velocities'][i]))
            frame_pairs.extend(norm(sub(frame['positions'][i],frame['positions'][j])) for j in frame['active'][k+1:])
    _,tube=load_benchmark(branch/'benchmark.json');end=tube.points[-1];tangent=unit(sub(end,tube.points[-2]));arc=0;crossings=[]
    for a,b in zip(tube.points,tube.points[1:]):
        da,db=dot(sub(a,end),tangent),dot(sub(b,end),tangent)
        if da<0<=db:
            fraction=-da/(db-da)
            crossings.append(arc+fraction*norm(sub(b,a)))
        arc+=norm(sub(b,a))
    mismatches=[name for name,digest in json.load(open(branch/'source_hashes.json')).items() if hashlib.sha256(Path(name).read_bytes()).hexdigest()!=digest]
    # Analytic cone: r(x)=5+0.5x. Radial clearance is not surface distance.
    cone=Tube([[0,0,0],[100,0,0]],[5,55]);point=[50,25,0]
    radial=cone.project(point)['radial_clearance'];surface=radial/math.sqrt(1+.5**2)
    findings=dict(radial_clearance_counterexample=dict(radial=radial,true_lateral_surface_distance=surface,
                                                     meaning='Radial clearance >=5 does not imply Euclidean tube-surface distance >=5.'))
    data=dict(result_extrema=extrema,short_prefix_identical=short==full[:len(short)],prefix_rows=len(short),source_hash_mismatches=mismatches,
              frames=dict(min_pair=min(frame_pairs),max_speed=max(frame_speeds),final_active=trajectory['frames'][-1]['active']),
              centerline_terminal_plane_crossing_arcs=crossings,tube_length=arc,solver=numerical_checks(),findings=findings,
              scope='Audit samples and saved results, not proof of continuous safety or paper-exact reproduction.')
    write_json(out/'audit.json',data)
    assert all(r['matches'] for r in extrema.values()) and data['short_prefix_identical'] and not mismatches
    assert all(case['passed'] for case in data['solver'].values())
    print(json.dumps(data,indent=2))

if __name__=='__main__':main()
