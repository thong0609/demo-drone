"""Paired experiment report; preserves failures and separates numerical from plant safety."""
import html
from pathlib import Path
from .parameters import saved_parameters


def export_comparison(out,results,hashes):
    parameters=saved_parameters(out)
    fields=[('completed','UAV hoàn thành'),('finish_time','Thời gian hoàn thành (s)'),
            ('min_pair_distance','Khoảng cách cặp nhỏ nhất (m)'),('min_boundary_clearance','Khoảng hở biên cục bộ nhỏ nhất (m)'),
            ('max_pair_distance','Khoảng cách cặp lớn nhất (m)'),('max_speed','Vận tốc thực lớn nhất (m/s)'),
            ('max_command_speed','Lệnh vận tốc lớn nhất (m/s)'),('max_input','Input component lớn nhất (N)'),
            ('failure_time','Thời điểm lỗi (s)'),('error','Lỗi kết thúc'),('success','Đạt mọi tiêu chí')]
    def format_value(value):
        if value is None:return '—'
        if isinstance(value,float):return f'{value:.5f}'
        return html.escape(str(value))
    header=''.join(f'<th>{html.escape(name)}</th>' for name in results)
    table_rows=''.join('<tr><td>'+label+'</td>'+''.join('<td>'+format_value(s.get(key))+'</td>' for s in results.values())+'</tr>' for key,label in fields)
    links=''.join(f'<li>{html.escape(name)}: <a href="{name}/report.html">Báo cáo</a> · <a href="{name}/replay.html">Replay 3D</a> · <a href="{name}/metrics.csv">CSV</a> · <a href="{name}/summary.json">JSON</a></li>' for name in results)
    dmpc_result=results.get('dmpc',{})
    scope_note=('DMPC đã hoàn thành nhiệm vụ; xem riêng các ngưỡng safety và timing trong bảng.'
                if dmpc_result.get('all_completed') else
                'Run DMPC dừng trước khi hoàn thành: các extrema chỉ thuộc đoạn đã chạy, không chứng nhận an toàn cả nhiệm vụ.')
    solver=dmpc_result.get('solver',{})
    diagnostics='<br>'.join(f'{html.escape(k)}: {format_value(v)}' for k,v in solver.items())
    import csv
    import json
    period=.2
    settings=out/'inputs/dmpc_config.json'
    if settings.exists():period=json.loads(settings.read_text())['dmpc']['update_period']
    plots=[];series=[]
    for name in results:
        path=out/name/'metrics.csv'
        if path.exists():
            with path.open() as f:
                rows=[{k:float(v) if v else None for k,v in row.items()} for row in csv.DictReader(f)]
            if rows:series.append((name,rows))
    figures=out/'figures';figures.mkdir(exist_ok=True)
    for key,title,limit in [('min_pair_distance','Minimum inter-UAV distance (m)',10),('min_boundary_clearance','Minimum local tube clearance (m)',5),('max_pair_distance','Maximum inter-UAV distance (m)',150),('max_speed','Maximum actual speed (m/s)',11),('max_command_speed','Maximum command speed (m/s)',11),('max_input','Maximum input component (N)',30)]:
        plot=overlay_chart(series,key,title,limit)
        if plot:
            (figures/f'{key}.svg').write_text(plot)
            plots.append(f'<img style="width:100%;margin-top:16px" src="figures/{key}.svg" alt="{title}">')
    plots=''.join(plots)
    text=f'''<!doctype html><html lang="vi"><meta charset="utf-8"><title>Outer loop comparison</title><style>body{{max-width:1100px;margin:40px auto;padding:0 24px;font:16px system-ui;color:#193444;background:#f3f7f8}}td,th{{padding:12px;border-bottom:1px solid #c8d6dc;text-align:left}}table{{border-collapse:collapse;background:white;width:100%}}p{{line-height:1.7}}a{{color:#107b88}}code{{word-break:break-all}}</style>
<h1>Vector field ↔ DMPC · Tube tĩnh</h1><p>Chỉ thay outer loop. Cùng tube, toàn bộ initial state, inner loop, plant, nhiễu và ngưỡng đánh giá. DMPC cập nhật {period:g} s; vector field cập nhật 0.02 s. Đây là phiên bản DMPC riêng của benchmark, không tái hiện nguyên bản một công trình khác.</p>
{parameters}<h2>Kết quả so sánh</h2><table><tr><th>Chỉ số</th>{header}</tr>{table_rows}</table><ul>{links}</ul>
{plots}<h2>DMPC diagnostics</h2><p>{diagnostics}</p><p><a href="dmpc/solver.jsonl">Local QP logs</a> · <a href="dmpc/prediction_errors.json">Prediction errors</a> · <a href="inputs/predictor_calibration.json">Calibration</a> · <a href="comparison.json">Comparison JSON</a> · <a href="invariants.json">Shared input checks</a> · <a href="dmpc/timing_and_prediction_summary.json">Timing details</a></p>
<p>{scope_note}</p><p>DMPC dùng cruise reference 9 m/s, horizon 3 s và các margins trong cấu hình; baseline có guidance bão hòa 11 m/s. Đây là một điểm vận hành với đánh đổi tốc độ/an toàn, chưa đủ để kết luận ưu thế tổng quát của thuật toán.</p><p>Giới hạn dự báo không bảo đảm plant thực an toàn khi predictor chỉ gần đúng. Norm constraints dùng đa diện nội tiếp; tube cuts là xấp xỉ cục bộ và được kiểm tra lại. Failure/infeasibility và deadline misses được giữ nguyên, không clip vận tốc thực.</p>
<p>Benchmark hash: <code>{hashes['benchmark']}</code><br>Initial state hash: <code>{hashes['initial_state']}</code></p></html>'''
    (out/'report.html').write_text(text)


def overlay_chart(series,key,title,threshold):
    """Two algorithms on common axes, preserving unequal run durations."""
    import math
    W,H=760,300;left,right,top,bottom=70,25,52,42
    values=[r[key] for _,rows in series for r in rows if r[key] is not None]
    if not values:return ''
    lo=min(min(values),threshold);hi=max(max(values),threshold)
    pad=max(.1,.12*(hi-lo));lo=max(0,lo-pad);hi+=pad
    end=max(rows[-1]['time'] for _,rows in series if rows)
    end=max(end,1)
    def x(t):return left+t/end*(W-left-right)
    def y(v):return H-bottom-(v-lo)/(hi-lo)*(H-top-bottom)
    parts=[f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}"><rect width="100%" height="100%" fill="white"/>',
           f'<text x="{left}" y="23" font-size="16" font-family="sans-serif">{html.escape(title)}</text>']
    for i in range(5):
        value=lo+(hi-lo)*i/4;yy=y(value)
        parts.append(f'<path d="M{left},{yy}H{W-right}" stroke="#e3e9ec"/><text x="{left-8}" y="{yy+4}" text-anchor="end" font-size="11">{value:.2f}</text>')
    for i in range(6):
        t=end*i/5
        parts.append(f'<text x="{x(t)}" y="{H-bottom+20}" text-anchor="middle" font-size="11">{t:.0f}</text>')
    parts.append(f'<path d="M{left},{y(threshold)}H{W-right}" stroke="#ba4141" stroke-dasharray="6 4"/>')
    for index,(name,rows) in enumerate(series):
        color=['#187b8d','#c27822'][index%2];path='';drawing=False
        for row in rows:
            value=row[key]
            if value is None:drawing=False;continue
            path+=('L' if drawing else 'M')+f'{x(row["time"]):.2f},{y(value):.2f} ';drawing=True
        parts.append(f'<path d="{path}" fill="none" stroke="{color}" stroke-width="1.7"/><text x="{left+index*230}" y="41" fill="{color}" font-size="12" font-family="sans-serif">{name}</text>')
    parts.append(f'<text x="{W/2}" y="{H-4}" text-anchor="middle" font-size="12">Time (s)</text></svg>')
    return ''.join(parts)
