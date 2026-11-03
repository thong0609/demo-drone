"""Standalone SVG scientific plots, trajectory replay, and experiment report."""
import html
import json
import math
from pathlib import Path
from .export import write_viewer
from .snapshots import snapshot
from .parameters import render_parameters


def chart(history,key,title,threshold=None):
    W,H=640,270;left,right,top,bottom=64,24,42,44
    values=[r[key] for r in history if r[key] is not None]
    if not values: return '<svg xmlns="http://www.w3.org/2000/svg"/>'
    ymin=min(values);ymax=max(values)
    if threshold is not None: ymin=min(ymin,threshold);ymax=max(ymax,threshold)
    padding=max(.1,(ymax-ymin)*.12); ymin=max(0,ymin-padding);ymax+=padding
    xmax=max(history[-1]['time'],1)
    def x(t):return left+t/xmax*(W-left-right)
    def y(v):return H-bottom-(v-ymin)/(ymax-ymin)*(H-top-bottom)
    lines=[f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}"><rect width="100%" height="100%" fill="white"/>',f'<text x="{left}" y="24" font-family="sans-serif" font-size="16">{html.escape(title)}</text>']
    for i in range(5):
        v=ymin+(ymax-ymin)*i/4;yy=y(v)
        lines += [f'<path d="M{left},{yy}H{W-right}" stroke="#e0e5e9"/>',f'<text x="{left-8}" y="{yy+4}" text-anchor="end" font-size="11" font-family="sans-serif">{v:.2f}</text>']
    for i in range(6):
        t=xmax*i/5;xx=x(t)
        lines.append(f'<text x="{xx}" y="{H-bottom+20}" text-anchor="middle" font-size="11" font-family="sans-serif">{t:.0f}</text>')
    if threshold is not None:
        yy=y(threshold);lines.append(f'<path d="M{left},{yy}H{W-right}" stroke="#d64c3e" stroke-width="1.5" stroke-dasharray="6 4"/><text x="{W-right}" y="{yy-5}" text-anchor="end" font-size="11" fill="#b23c31">limit {threshold:g}</text>')
    path='';pen=False
    for row in history:
        value=row[key]
        if value is None:pen=False;continue
        path+=('L' if pen else 'M')+f'{x(row["time"]):.2f},{y(value):.2f} ';pen=True
    lines.append(f'<path d="{path}" fill="none" stroke="#147d91" stroke-width="1.7"/><text x="{W/2}" y="{H-6}" text-anchor="middle" font-size="12" font-family="sans-serif">Time (s)</text></svg>')
    return ''.join(lines)


def export_results(payload,tube,frames,history,summary,cfg,out):
    specs=[('min_pair_distance','Minimum inter-UAV distance (m)',2*cfg['outer']['rs']),
           ('min_boundary_clearance','Minimum local tube clearance (m)',cfg['outer']['rs']),
           ('max_pair_distance','Maximum inter-UAV distance (m)',cfg['outer']['lm']),
           ('max_speed','Maximum actual speed (m/s)',cfg['outer']['vm']),
           ('max_command_speed','Maximum commanded speed (m/s)',cfg['outer']['vm']),
           ('max_input','Maximum input component (N)',max(cfg['dynamics']['input_limit'])),
           ('tracking_rmse','Velocity tracking RMSE per axis (m/s)',None),
           ('disturbance_rmse','Observer residual estimate RMSE per axis (m/s²)',None)]
    controller_name=cfg.get('controller_name','Enhanced vector field')
    figures=out/'figures';figures.mkdir(exist_ok=True)
    for key,title,threshold in specs:(figures/f'{key}.svg').write_text(chart(history,key,title,threshold))
    snapshots=[]
    for requested in (0,25,50):
        frame=min(frames,key=lambda f:abs(f['time']-requested))
        name=f'snapshot_{requested:02d}s.svg'
        (figures/name).write_text(snapshot(payload,tube,frame))
        snapshots.append(name)
    write_viewer(payload,tube,out/'environment.html')
    write_viewer(payload,tube,out/'replay.html')
    text=(out/'replay.html').read_text()
    text=text.replace('Curve tube · Benchmark 01',controller_name+' · Swarm replay').replace('Benchmark 01',controller_name+' / 27 UAVs')
    text=text.replace('Vị trí trên đường tâm','Thời gian mô phỏng')
    text=text.replace('<button id="reset">','<button id="play">▶ Chạy</button> <button id="reset">')
    text=text.replace('Tube là đường tham chiếu cố định, chưa chạy bộ lập kế hoạch hay điều khiển.',controller_name+'. Tube và plant cố định; UAV hoàn thành được đưa ra khỏi miền mô phỏng.')
    text=text.replace('const data=', 'const replay='+json.dumps(frames)+';\nconst data=',1)
    text=text.replace("el('progress').max=points.length-1;","el('progress').max=replay.length-1;")
    start=text.index("let k=+el('progress').value;")
    end=text.index("function resize()",start)
    text=text[:start]+'''let frame=replay[+el('progress').value];
let live=new Set(frame.active);
for(let i=0;i<frame.positions.length;i++){
 if(!live.has(i))continue;
 let trail=replay.slice(0,+el('progress').value+1).filter((f,j)=>j%4===0).map(f=>f.positions[i]);
 line(trail,'rgba(110,170,225,.4)',.7);
 let p=frame.positions[i],q=project(p),s=Math.min(W/850,H/680)*zoom;
 ctx.beginPath();ctx.arc(q[0],q[1],5*s,0,2*Math.PI);ctx.fillStyle='rgba(255,212,130,.75)';ctx.fill();ctx.strokeStyle='#ffdaa2';ctx.lineWidth=.6;ctx.stroke();
}
label(config.start,'START','#dceff3');label(config.goal,'GOAL','#dceff3');
el('pct').textContent=frame.time.toFixed(1)+' s';el('sample').textContent='Đang bay: '+frame.active.length+' / 27 · Đã ra đích: '+(27-frame.active.length);}
let playing=false,lastFrame=0;
el('play').onclick=()=>{playing=!playing;el('play').textContent=playing?'Ⅱ Dừng':'▶ Chạy';};
function animate(t){if(playing && t-lastFrame>50){let slider=el('progress');if(+slider.value>=replay.length-1){slider.value=0;}else slider.value=+slider.value+1;lastFrame=t;draw();}requestAnimationFrame(animate)}
requestAnimationFrame(animate);
''' +text[end:]
    (out/'replay.html').write_text(text)
    initial_path=out/'initial_state.json'
    initial=json.loads(initial_path.read_text()) if initial_path.exists() else None
    parameters=render_parameters(payload,cfg,initial,cfg.get('dmpc'))
    status='ĐẠT' if summary['success'] else 'CẦN XEM CÁC RÀNG BUỘC'
    cards=''.join(f'<figure><img src="figures/{key}.svg" alt="{html.escape(title)}"/></figure>' for key,_,_ in specs)
    cards=''.join(f'<figure><img src="figures/{name}" alt="Swarm snapshot"/></figure>' for name in snapshots)+cards
    rows=''.join(f'<tr><td>{html.escape(k)}</td><td>{html.escape(str(v))}</td></tr>' for k,v in summary.items() if k not in ('completion_times','metrics_scope'))
    body=f'''<!doctype html><html lang="vi"><meta charset="utf-8"><title>Paper V-B — results</title><style>body{{font:15px system-ui;background:#f1f5f7;color:#172c3a;max-width:1300px;margin:32px auto;padding:0 24px}}a{{color:#147d91}}.plots{{display:grid;grid-template-columns:repeat(auto-fit,minmax(430px,1fr));gap:16px}}figure{{margin:0;background:white;border-radius:10px;overflow:hidden}}img{{width:100%}}td{{padding:7px 12px;border-bottom:1px solid #ccc}}table{{border-collapse:collapse}}p{{line-height:1.7}}</style>
<h1>Thí nghiệm V-B · {status}</h1><p>27 UAV · benchmark tự dựng · {html.escape(controller_name)} / actor–critic / LDO; tube cố định.</p>
<p><a href="replay.html">Mở hoạt ảnh 3D</a> · <a href="environment.html">Xem tube</a> · <a href="metrics.csv">Dữ liệu CSV</a> · <a href="resolved_config.json">Cấu hình</a> · <a href="summary.json">Kết quả JSON</a></p>
<p>Hoàn thành: <b>{summary['completed']}/27</b>. Số liệu là kết quả trên bản đồ tự dựng, không phải tái tạo chính xác Hình 8. Ngưỡng đỏ thể hiện giới hạn trong cấu hình. Vi phạm vận tốc thực vẫn được báo cáo dù lệnh vận tốc đã bão hòa.</p>
{parameters}<h2>Kết quả mô phỏng</h2><div class="plots">{cards}</div><h2>Thông số và kiểm tra</h2><table>{rows}</table><p>{html.escape(summary['metrics_scope'])}</p><p>Định nghĩa biên tube dùng khoảng cách hướng kính cục bộ; đây không phải SDF chính xác của toàn bộ bề mặt cong. Xem docs/PAPER_VB_IMPLEMENTATION.md để biết các giả định và giới hạn.</p></html>'''
    (out/'report.html').write_text(body)
