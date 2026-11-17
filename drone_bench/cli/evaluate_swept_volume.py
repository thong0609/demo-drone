"""Paired exploratory evaluation; includes every failure and common-horizon checks."""
import argparse
import copy
import csv
import html
import hashlib
import json
import math
import platform
import statistics
import time
from pathlib import Path
from ..algorithms.planners.paper_tube import build_paper_tube
from ..algorithms.planners.swept_volume import update_tube, check_tube
from ..algorithms.planners.tube_quality import shape_metrics, displacement
from ..environment.world import Environment
from ..environment.swept import SweptEnvironment
from ..io.benchmark import write_json
from ..visualization.swept_replay import write_replay
from .plan_swept_volume import geometry


MODES=('baseline','balanced_only','balanced_fixed_seed','improved','instantaneous','static_tube')


def settings(base,mode):
    cfg=copy.deepcopy(base)
    for key in ('duration_policy','reuse_waypoints','max_length_inflation','seed_policy'):cfg.pop(key,None)
    if mode!='baseline' and mode!='static_tube':cfg['duration_policy']='balanced'
    if mode in ('improved','instantaneous'):
        cfg.update(reuse_waypoints=True,max_length_inflation=1.1)
    return cfg


def cases(base, direction_catalog=None):
    result=[]
    for name in ('crossing','large','fast','multiple','static_clutter','horizon_1','horizon_6','blocked','future_start'):
        c=copy.deepcopy(base);c['update_times']=[1,2];c['planner']['iterations']=150
        if name=='large':c['obstacles'][0]['radius']=18
        if name=='fast':c['obstacles'][0]['velocity']=[0,-45,0]
        if name.startswith('horizon_'):c['horizon']=int(name.split('_')[1])
        if name=='multiple':
            c['obstacles'].append(dict(id='third',shape='sphere',center=[320,220,190],
                                       radius=7,velocity=[0,10,-8],timestamp=0))
        if name=='static_clutter':c['world']['obstacles']=[dict(type='sphere',center=[330,260,90],radius=15)]
        if name in ('blocked','future_start'):
            start=c['world']['start'];future=name=='future_start'
            c['obstacles']=[dict(id='blocker',shape='sphere',center=[start[0],start[1]+(80 if future else 0),start[2]],
                                 radius=4,velocity=[0,-20,0] if future else [0,0,0],timestamp=0)]
        result.append((name,c))
    catalog=direction_catalog or Path(__file__).resolve().parents[2]/'configs/experiments/swept_direction_cases.json'
    for item in json.loads(Path(catalog).read_text())['cases']:
        c=copy.deepcopy(base);c['update_times']=[1,2];c['planner']['iterations']=150
        c['obstacles']=item['obstacles'];c['case_label']=item['label'];c['case_description']=item['description']
        result.append((item['name'],c))
    if len({name for name,_ in result})!=len(result):raise ValueError('Duplicate case names')
    return result


def evaluate(config,out,seeds=(7,17,29),direction_catalog=None):
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    root=Path(__file__).resolve().parents[2]
    library=root/'drone_bench'
    paths=list((library/'algorithms/planners').glob('*.py'))+list((library/'environment').glob('*.py'))+list((library/'tubes').glob('*.py'))+list((library/'common').glob('*.py'))+[Path(__file__).resolve(),library/'visualization/swept_replay.py',library/'visualization/swept_replay.html',library/'cli/plan_swept_volume.py']
    write_json(out/'source_hashes.json',{str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths})
    matrix=cases(config,direction_catalog);rows=[];details=[]
    write_json(out/'protocol.json',dict(seeds=list(seeds),modes=MODES,cases=[dict(name=n,config=c) for n,c in matrix],
                                      evaluation='paired exploratory suite; no separate held-out tuning set',
                                      common_horizon='each case uses its own configured H, including horizon ablations',platform=platform.platform(),processor=platform.processor(),
                                      python=platform.python_version(),timing='wall update including certification; common evaluator timed separately'))
    for name,c in matrix:
        for seed in seeds:
            for mode in MODES:
                cfg=settings(c['planner'],mode);previous=None;initial_error=None;replay_updates=[]
                try:previous,_=build_paper_tube(Environment(c['world']),cfg,seed)
                except (RuntimeError,ValueError,ArithmeticError) as error:initial_error=str(error)
                initial_geometry=geometry(previous) if previous else None
                for index,now in enumerate(c['update_times']):
                    observations=[]
                    for o in c['obstacles']:
                        item=copy.deepcopy(o);item['center']=[p+v*(now-o['timestamp']) for p,v in zip(o['center'],o['velocity'])];item['timestamp']=now;observations.append(item)
                    full=SweptEnvironment(c['world'],observations,now,c['horizon'],c['prediction_margin'])
                    env=full if mode!='instantaneous' else SweptEnvironment(c['world'],observations,now,1e-6,c['prediction_margin'])
                    before=previous
                    if mode=='static_tube':
                        tube=previous;report=dict(status='static_reference',accepted=tube is not None,elapsed_seconds=0.,planning=None)
                    else:
                        current_seed=seed if mode in ('balanced_fixed_seed','improved','instantaneous') else seed+index
                        tube,report=update_tube(env,cfg,current_seed,previous,c['deadline_seconds'])
                    previous=tube
                    check_started=time.perf_counter();valid=False;error=None;certificate=None
                    if tube:
                        try:certificate=check_tube(tube,full,c['world']['tube']['obstacle_margin'],cfg['radius_floor']);valid=True
                        except ValueError as e:error=str(e)
                    else:error=report.get('error','No candidate')
                    check_seconds=time.perf_counter()-check_started
                    report.update(common_horizon_certified=valid,common_horizon_error=error,
                                  update_index=index,observations=observations,snapshot=env.describe())
                    replay_updates.append(dict(report=report,tube=geometry(tube) if tube else None))
                    path=(report.get('planning') or {}).get('waypoints')
                    if tube and not path and hasattr(tube,'curve'):
                        path=[tube.sample(s,0)[0] for s in range(tube.spans)]+[tube.sample(tube.spans-1,1)[0]]
                    metrics=shape_metrics(tube,path) if tube else {}
                    change=displacement(before,tube) if before and tube else {}
                    row=dict(case=name,seed=seed,mode=mode,time=now,status=report['status'],
                             planned=report['accepted'],common_horizon_certified=valid,
                             deadline_missed=report['elapsed_seconds']>c['deadline_seconds'],
                             update_seconds=report['elapsed_seconds'],evaluator_seconds=check_seconds,
                             length=metrics.get('length'),length_inflation=metrics.get('length_inflation'),
                             backward_distance=metrics.get('backward_distance_along_start_goal'),
                             max_curvature=metrics.get('sampled_max_curvature'),min_radius=metrics.get('min_radius'),
                             change_mean=change.get('mean'),change_max=change.get('maximum'),
                             error=error,initial_error=initial_error)
                    rows.append(row);details.append(dict(row=row,report=report,metrics=metrics,common_certificate=certificate))
                replay_dir=out/'replays'/name/f'seed_{seed}'
                replay_dir.mkdir(parents=True,exist_ok=True)
                replay_config=copy.deepcopy(c);replay_config['planner']=cfg;replay_config['seed']=seed
                write_replay(replay_dir/(mode+'.html'),replay_config,replay_updates,initial_geometry,
                             dict(case=name,seed=seed,mode=mode,scope='planner-only; see common-horizon result for every update',initial_error=initial_error))
        print('Evaluated '+name,flush=True)
        # Keep partial results reviewable if a later case is interrupted.
        write_json(out/'details.json',details)
    with (out/'metrics.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    aggregate=[]
    for mode in MODES:
        group=[r for r in rows if r['mode']==mode];latencies=sorted(r['update_seconds'] for r in group)
        def mean(key):
            values=[r[key] for r in group if r['common_horizon_certified'] and r[key] is not None]
            return statistics.mean(values) if values else None
        aggregate.append(dict(mode=mode,updates=len(group),planned=sum(r['planned'] for r in group),
                              certified=sum(r['common_horizon_certified'] for r in group),
                              deadline_misses=sum(r['deadline_missed'] for r in group),
                              p95_seconds=latencies[math.ceil(.95*len(latencies))-1],max_seconds=max(latencies),
                              mean_length_certified=mean('length'),mean_inflation_certified=mean('length_inflation'),
                              mean_change_certified=mean('change_mean'),
                              backward_cases=sum((r['backward_distance'] or 0)>1e-3 for r in group),
                              metric_scope='shape averages conditional on common-horizon certificate; backward count includes all planned tubes'))
    write_json(out/'summary.json',aggregate)
    paired=[]
    for mode in MODES[1:]:
        pairs=[]
        for row in rows:
            if row['mode']!=mode or not row['common_horizon_certified']:continue
            reference=next(r for r in rows if r['mode']=='baseline' and all(r[k]==row[k] for k in ('case','seed','time')))
            if reference['common_horizon_certified']:pairs.append((reference,row))
        paired.append(dict(mode=mode,matched_certified_updates=len(pairs),
                           mean_length_delta=statistics.mean(b['length']-a['length'] for a,b in pairs) if pairs else None,
                           mean_change_delta=statistics.mean(b['change_mean']-a['change_mean'] for a,b in pairs if a['change_mean'] is not None and b['change_mean'] is not None) if pairs else None))
    variability=[]
    for name,_ in matrix:
        for mode in MODES:
            group=[r for r in rows if r['case']==name and r['mode']==mode and r['time']==1 and r['common_horizon_certified']]
            values=[r['length'] for r in group]
            variability.append(dict(case=name,mode=mode,certified_seeds=len(values),
                                    length_min=min(values,default=None),length_max=max(values,default=None),
                                    length_std=statistics.pstdev(values) if values else None))
    write_json(out/'paired_comparison.json',paired)
    write_json(out/'seed_variability.json',variability)
    case_summaries=[]
    for name,c in matrix:
        for mode in MODES:
            group=[r for r in rows if r['case']==name and r['mode']==mode]
            case_summaries.append(dict(case=name,mode=mode,updates=len(group),planned=sum(r['planned'] for r in group),
                                       certified=sum(r['common_horizon_certified'] for r in group),
                                       statuses={status:sum(r['status']==status for r in group) for status in sorted({r['status'] for r in group})},
                                       max_seconds=max(r['update_seconds'] for r in group)))
    write_json(out/'case_summary.json',case_summaries)
    write_full_report(out,matrix,rows,aggregate,case_summaries,seeds)
    return aggregate


def write_full_report(out,matrix,rows,aggregate,case_summaries,seeds):
    """Every case, every seed/mode/update and failure is reachable from one report."""
    labels={'baseline':'Ban đầu','balanced_only':'Chỉ đổi thời lượng','balanced_fixed_seed':'Thêm seed cố định',
            'improved':'Cải tiến','instantaneous':'Chỉ né vị trí hiện tại','static_tube':'Tube tĩnh'}
    esc=lambda value:html.escape(str(value))
    style='body{font:15px system-ui;background:#101a2a;color:#dfe8f6;margin:0}main{max-width:1450px;margin:auto;padding:24px}a{color:#7bcaff}section{background:#182538;border:1px solid #34455e;padding:20px;margin:20px 0;border-radius:10px}table{border-collapse:collapse;width:100%;font-size:13px}td,th{padding:9px;text-align:left;border-bottom:1px solid #34455e}.scroll{overflow:auto}.good{color:#8be3b9}.bad{color:#ffad91}summary{cursor:pointer;margin:12px 0}code{color:#acdbff}.nav{display:flex;gap:12px;flex-wrap:wrap}.muted{color:#adbed3}'
    parts=['<!doctype html><html lang="vi"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Đánh giá đầy đủ planner tube động</title><style>'+style+'</style><main><h1>Đánh giá planner qua nhiều hướng vật cản</h1>',
           f'<p>{len(matrix)} tình huống × {len(seeds)} seed × 2 thời điểm × {len(MODES)} phương án = <strong>{len(rows)} lượt đánh giá</strong>. Có đủ kết quả và replay cho mọi trường hợp, kể cả thất bại.</p>',
           '<p class="muted">Snapshot: dữ liệu tại t=1 và 2 s. Seed: chuỗi lấy mẫu RRT*. Đạt: tube được chứng nhận tránh vùng quét trong horizon chung. Không đạt chứng nhận không tự chứng minh va chạm hoặc nhiệm vụ vô nghiệm. Các lượt trong một chuỗi có tương quan; đây là bộ thử tổng hợp, chưa phải đánh giá bay thực hoặc tập held-out.</p>',
           '<div class="nav">'+''.join(f'<a href="#{esc(n)}">{i+1}. {esc(c.get("case_label",n))}</a>' for i,(n,c) in enumerate(matrix))+'</div>',
           '<section><h2>Tổng hợp toàn bộ</h2><div class="scroll"><table><tr><th>Phương án</th><th>Candidate</th><th>Đạt horizon chung</th><th>p95 tính (ms)</th><th>Quá deadline</th></tr>']
    for r in aggregate:parts.append(f'<tr><td>{labels[r["mode"]]}</td><td>{r["planned"]}/{r["updates"]}</td><td>{r["certified"]}/{r["updates"]}</td><td>{r["p95_seconds"]*1000:.1f}</td><td>{r["deadline_misses"]}</td></tr>')
    parts.append('</table></div><p class="muted">Cùng budget 150 RRT* iterations. p95 chỉ tính update planner, chưa gồm dựng snapshot hoặc hệ cảm biến; tube tĩnh không có chi phí cập nhật online. Các chỉ số chiều dài chi tiết chỉ tính khi có candidate; xem CSV và kết quả từng dòng.</p><p><a href="metrics.csv">Tải CSV đầy đủ</a> · <a href="protocol.json">Cấu hình mọi tình huống</a> · <a href="paired_comparison.json">So sánh theo cặp</a> · <a href="seed_variability.json">Độ biến động qua seed</a></p></section>')
    for index,(name,c) in enumerate(matrix):
        group=[r for r in rows if r['case']==name]
        parts.append(f'<section id="{esc(name)}"><h2>{index+1}. {esc(c.get("case_label",name))}</h2><p><code>{esc(name)}</code> · Horizon {c["horizon"]} s · Các mốc: {esc(c["update_times"])}</p><p>{esc(c.get("case_description","Biến thể kiểm tra kích thước, vận tốc, hình học hoặc giới hạn của vùng quét."))}</p>')
        parts.append('<div class="scroll"><table><tr><th>Vật cản</th><th>Tâm lúc t=0 (m)</th><th>Vận tốc XYZ (m/s)</th><th>Tốc độ (m/s)</th><th>Bán kính (m)</th></tr>')
        for o in c['obstacles']:parts.append(f'<tr><td>{esc(o["id"])}</td><td>{esc(o["center"])}</td><td>{esc(o["velocity"])}</td><td>{math.sqrt(sum(v*v for v in o["velocity"])):.2f}</td><td>{o["radius"]}</td></tr>')
        parts.append('</table></div><p>Vật cản tĩnh trong map: '+esc(len(c['world']['obstacles']))+'. Thông số đầy đủ nằm trong protocol.json.</p>')
        parts.append('<div class="scroll"><table><tr><th>Phương án</th><th>Tạo được</th><th>Đạt horizon chung</th><th>Trạng thái / số lần</th><th>Max thời gian (ms)</th></tr>')
        for r in case_summaries:
            if r['case']==name:parts.append(f'<tr><td>{labels[r["mode"]]}</td><td>{r["planned"]}/{r["updates"]}</td><td>{r["certified"]}/{r["updates"]}</td><td>{esc(r["statuses"])}</td><td>{r["max_seconds"]*1000:.1f}</td></tr>')
        parts.append('</table></div><h3>Replay của từng lần chạy</h3><div class="scroll"><table><tr><th>Seed</th>'+''.join('<th>'+labels[m]+'</th>' for m in MODES)+'</tr>')
        for seed in seeds:parts.append('<tr><td>'+str(seed)+'</td>'+''.join(f'<td><a href="replays/{esc(name)}/seed_{seed}/{m}.html">Mở replay</a></td>' for m in MODES)+'</tr>')
        parts.append('</table></div><details><summary>Xem mọi kết quả, kể cả ca không đạt</summary><div class="scroll"><table><tr><th>Seed</th><th>Phương án</th><th>t (s)</th><th>Trạng thái</th><th>Đạt horizon?</th><th>Độ dài (m)</th><th>Cong max lấy mẫu (1/m)</th><th>Lỗi/lý do</th></tr>')
        for r in group:
            f=lambda v:'—' if v is None else f'{v:.3f}'
            parts.append(f'<tr><td>{r["seed"]}</td><td>{labels[r["mode"]]}</td><td>{r["time"]}</td><td>{esc(r["status"])}</td><td class="'+('good' if r['common_horizon_certified'] else 'bad')+'">'+('Đạt' if r['common_horizon_certified'] else 'Không đạt')+f'</td><td>{f(r["length"])}</td><td>{f(r["max_curvature"])}</td><td>{esc(r["error"] or r["initial_error"] or "—")}</td></tr>')
        parts.append('</table></div></details></section>')
    parts.append('</main></html>');(out/'report.html').write_text(''.join(parts))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',default='configs/experiments/swept_volume_replay.json')
    parser.add_argument('--output',default='artifacts/swept_volume_direction_evaluation')
    parser.add_argument('--directions',help='JSON catalog of direction cases; defaults to built-in catalog')
    args=parser.parse_args()
    print(json.dumps(evaluate(json.loads(Path(args.config).read_text()),args.output,direction_catalog=args.directions),indent=2))


if __name__=='__main__':main()
