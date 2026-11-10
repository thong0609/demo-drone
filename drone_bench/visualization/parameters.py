"""Human-readable parameters from saved experiment inputs, never live configs."""
import html
import json
from pathlib import Path


def display(value):
    if isinstance(value, float):
        return f'{value:.6g}'
    if isinstance(value, (list, tuple)):
        return '(' + ', '.join(display(v) for v in value) + ')'
    if isinstance(value, bool):
        return 'Có' if value else 'Không'
    return str(value)


def table(headers, rows):
    head=''.join(f'<th>{html.escape(str(v))}</th>' for v in headers)
    body=''.join('<tr>'+''.join(f'<td>{html.escape(display(v))}</td>' for v in row)+'</tr>' for row in rows)
    return f'<div style="overflow-x:auto"><table><thead><tr>{head}</tr></thead><tbody>{body}</tbody></table></div>'


def render_parameters(payload, cfg, initial=None, dmpc=None):
    scenario=payload['scenario'];physical=cfg.get('dynamics',{});outer=cfg.get('outer',{})
    sim=cfg.get('simulation',{});inner=cfg.get('inner',{})
    bounds=scenario['bounds'];size=[b-a for a,b in zip(*bounds)]
    states=(initial or {}).get('states',[])
    count=len(states) if states else None
    paper=set(cfg.get('provenance',{}).get('paper_parameters',[]))
    def origin(path):return 'Theo paper trong cấu hình lưu' if path in paper else 'Thiết lập benchmark'
    rows=[('Kích thước world',size,'m','X × Y × Z; Z hướng lên',origin('scenario.bounds')),
          ('Giới hạn world',bounds,'m','Góc nhỏ nhất và lớn nhất; biên world được coi là vật cản','Thiết lập benchmark'),
          ('p_start',scenario['start'],'m','Gốc đường tâm; UAV khởi tạo ở các tiết diện phía trong tube, theo chiều tiến về đích',origin('scenario.start')),
          ('p_goal',scenario['goal'],'m','Điểm cuối đường tâm; đích nhiệm vụ là đĩa tiết diện cuối',origin('scenario.goal')),
          ('N',count if count is not None else 'Chưa lưu','UAV','Lấy từ toàn bộ initial states đã lưu','Dữ liệu lần chạy'),
          ('Seed',cfg.get('seed','Chưa lưu'),'—','Seed lập kế hoạch; tube đã cố định trong phép so sánh','Thiết lập benchmark')]
    parts=['<section id="experiment-parameters"><h2>Môi trường và điều kiện ban đầu</h2>',
           '<p>Các giá trị dưới đây được đọc từ snapshot đi kèm lần chạy, không lấy từ file cấu hình đang chỉnh sửa. Tọa độ theo hệ X/Y/Z, đơn vị mét.</p>',
           table(['Biến / thông số','Giá trị','Đơn vị','Ý nghĩa','Nguồn'],rows)]
    obstacles=[]
    for o in scenario.get('obstacles',[]):
        if o['type']=='box':
            obstacles.append((o['id'],'Hộp',o['min'],o['max'],'—',[b-a for a,b in zip(o['min'],o['max'])]))
        elif o['type']=='sphere':
            obstacles.append((o['id'],'Cầu',o['center'],'—',o['radius'],'—'))
    parts+=['<h3>Vật cản cố định</h3><p>Tọa độ và kích thước vật cản là benchmark tự dựng, không phải dữ liệu tác giả công bố trong Hình 7. Các giá trị trong bảng đều tính bằng mét.</p>',
            table(['ID','Loại','Góc min / tâm','Góc max','Bán kính','Kích thước hộp'],obstacles)]
    points=payload['tube']['points'];radii=payload['tube']['radii'];report=payload.get('report',{})
    tube_rows=[('Chiều dài đường tâm',report.get('length','Chưa lưu'),'m','Tube thực tế được dùng trong lần chạy'),
               ('Bán kính nhỏ nhất / lớn nhất',[min(radii),max(radii)],'m','Tính từ radii đã xuất, không dùng giới hạn thiết kế thay cho giá trị thật'),
               ('Bán kính thiết kế tối đa',scenario.get('tube',{}).get('max_radius','Chưa lưu'),'m','Giới hạn khi lập tube'),
               ('Số mẫu đường tâm',len(points),'điểm','Số điểm trong tube đã đóng băng'),
               ('Khoảng hở tube–vật cản/world được chứng nhận',report.get('certified_obstacle_and_world_margin','Chưa lưu'),'m','Khoảng hở của thể tích bao tube; khác khoảng hở UAV–biên tube')]
    boundary_note=('<p>Biên tube: khoảng cách Euclidean tới mặt bên đa thức, tìm cực tiểu số trên các span liên tục; đĩa đầu/cuối là cửa vào/ra. Đây không phải chứng nhận khoảng cách toàn cục bằng số học khoảng.</p>'
                   if 'curve' in payload['tube'] else
                   '<p>Tube lấy mẫu cũ: xem phạm vi metrics trong summary đi kèm; các artifacts lịch sử có thể dùng khoảng hở hướng kính cục bộ.</p>')
    parts+=['<h3>Tube tĩnh dùng chung</h3>',table(['Thông số','Giá trị','Đơn vị','Ý nghĩa'],tube_rows),boundary_note]
    limits=[]
    labels={'vm':('v_max','m/s','Ngưỡng tốc độ thực để chấm điểm và giới hạn lệnh chung'),
            'rs':('r_s','m','Bán kính an toàn UAV'),
            'ra':('r_a','m','Bán kính vùng tránh va chạm của vector field'),
            'ls':('l_s','m','Ngưỡng bắt đầu hút đàn trong vector field'),
            'lm':('l_m','m','Khoảng cách cặp lớn nhất được chấp nhận')}
    for key,(label,unit,meaning) in labels.items():
        if key in outer:limits.append((label,outer[key],unit,meaning,origin('outer.'+key)))
    if 'rs' in outer:limits.append(('d_min = 2 r_s',2*outer['rs'],'m','Khoảng cách cặp UAV nhỏ nhất khi chấm điểm','Suy ra từ r_s'))
    parts+=['<h3>Giới hạn an toàn và chuyển động</h3>',table(['Biến','Giá trị','Đơn vị','Ý nghĩa','Nguồn'],limits)]
    initial_rows=[('Bố trí khởi tạo',sim.get('initial_stations','Chưa lưu'),'m dọc đường tâm','Các tiết diện đặt UAV ban đầu'),
                  ('Khoảng cách lưới',sim.get('grid_spacing','Chưa lưu'),'m','Mỗi tiết diện dùng lưới 3 × 3 trong cấu hình hiện tại'),
                  ('Tốc độ khởi tạo cấu hình',sim.get('initial_speed','Chưa lưu'),'m/s','Bảng dưới là vận tốc thực đã lưu; không suy ra trạng thái bộ lọc từ giá trị này'),
                  ('dt plant / inner',sim.get('dt','Chưa lưu'),'s','Bước tích phân của cùng plant và inner loop'),
                  ('Thời gian mô phỏng tối đa',sim.get('duration','Chưa lưu'),'s','Giới hạn thời gian nhiệm vụ'),
                  ('Khoảng ghi frame',sim.get('record_interval','Chưa lưu'),'s','Replay ghi thưa hơn metrics; metrics được đo mỗi dt')]
    parts+=['<h3>Khởi tạo và thời gian mô phỏng</h3>',table(['Thông số','Giá trị','Đơn vị','Ý nghĩa'],initial_rows)]
    if states:
        agent_rows=[(i,s[:3],s[3:6],s[6:9] if len(s)>=9 else 'Chưa lưu') for i,s in enumerate(states)]
        parts+=['<details><summary><b>Vị trí, vận tốc và bộ lọc ban đầu của từng UAV</b> (bấm để mở)</summary>',
                '<p>ID bắt đầu từ 0. v(0) là vận tốc thực; ζ(0) là trạng thái bộ lọc lệnh của inner loop, có thể khác 0 dù UAV đứng yên. Các trạng thái thích nghi còn lại được giữ trong initial_state.json.</p>',
                table(['ID','p(0) — m','v(0) — m/s','ζ(0) — m/s'],agent_rows),'</details>']
    dynamics_labels={'mass':('m','kg','Khối lượng UAV'), 'drag':('τ_x, τ_y, τ_z','kg/s','Hệ số drag tuyến tính'),
                     'gravity':('g','m/s²','Gia tốc trọng trường'), 'input_limit':('u_max từng trục','N','Giới hạn component lực, không phải giới hạn norm tổng'),
                     'uncertainty_amplitude':('Biên độ bất định','m/s²','Biên độ hàm bất định trong plant'),
                     'disturbance_amplitude':('Biên độ nhiễu','m/s²','Biên độ nhiễu sinus trong plant')}
    dynamics_rows=[(label,physical[key],unit,meaning) for key,(label,unit,meaning) in dynamics_labels.items() if key in physical]
    parts+=['<details><summary><b>Mô hình UAV và nhiễu dùng chung</b></summary>',table(['Biến','Giá trị','Đơn vị','Ý nghĩa'],dynamics_rows),'</details>']
    inner_labels={'P':'Gain SPEM','C':'Gain LDO','K':'Gain bám vận tốc','gamma_a':'Gain học actor','gamma_c':'Gain học critic',
                  'eta1':'Leakage actor','eta2':'Leakage critic','filter_tau':'Hằng số thời gian bộ lọc — s','Ka':'Gain hệ phụ bù bão hòa',
                  'robust_smoothing':'Tham số làm trơn robust term','Q':'Gain cập nhật thích nghi robust','eta3':'Leakage robust gain',
                  'rbf_width':'Độ rộng RBF','rbf_centers':'Tâm RBF trên đường chéo không gian đặc trưng','feature_scales':'Scale vector [v, ζ, ζ_dot]',
                  'basis_norm_bound':'Chặn norm hàm cơ sở NN sau chuẩn hóa','stability_witness':'Các hằng số Young kiểm tra (77); không phải gain outer'}
    parts+=['<details><summary><b>Thông số inner loop của lần chạy</b></summary>',
            table(['Tên trong cấu hình','Giá trị','Ý nghĩa','Nguồn'],[(k,v,inner_labels.get(k,'Tham số inner'),origin('inner.'+k)) for k,v in inner.items()]),'</details>']
    parts+=['<details><summary><b>Thông số outer vector field</b></summary>',
            table(['Tên trong cấu hình','Giá trị'],list(outer.items())),'</details>']
    if dmpc:
        dmpc_labels={'horizon':'Số prediction steps','prediction_dt':'Bước dự báo — s','move_blocks':'Số vector lệnh tối ưu',
                     'update_period':'Chu kỳ cập nhật outer — s','initial_hold':'Thời gian giữ lệnh khởi tạo chung — s',
                     'tau_velocity':'Hằng số bám tương đương, fit riêng — s','cruise_speed':'Tốc độ reference — m/s',
                     'speed_margin':'Dự phòng vận tốc dự báo — m/s','boundary_margin':'Dự phòng biên cục bộ — m',
                     'pair_margin':'Đệm cặp UAV ưu tiên — m','pair_hard_margin':'Đệm cặp UAV cứng trong forecast — m',
                     'pair_buffer_weight':'Trọng số phạt phục hồi buffer','pair_buffer_decay':'Thời gian phục hồi buffer — s',
                     'collision_activation':'Khoảng kích hoạt collision cuts — m','coordination_rounds':'Số vòng phối hợp tối đa',
                     'scp_iterations':'Số lần lặp SCP tối đa','qp_sweeps':'Ngân sách vòng lặp QP','qp_tolerance':'Tolerance residual QP',
                     'tube_facets':'Số mặt trên tiết diện xấp xỉ','command_rate':'Giới hạn thay đổi lệnh giữa move blocks — m/s²',
                     'position_weight':'Trọng số tracking vị trí','velocity_weight':'Trọng số tracking vận tốc',
                     'command_weight':'Trọng số độ lớn lệnh','slew_weight':'Trọng số thay đổi lệnh','cohesion_weight':'Trọng số cohesion mềm',
                     'validation_tolerance':'Tolerance kiểm tra dự báo','bias_filter_tau':'Hằng số lọc bias — s','bias_limit':'Bound bias estimator — m/s²'}
        horizon_time=dmpc['horizon']*dmpc['prediction_dt']
        derived=[('Horizon',horizon_time,'s'),
                 ('Giới hạn vận tốc forecast',outer['vm']-dmpc['speed_margin'],'m/s'),
                 ('Khoảng cách forecast ưu tiên',2*outer['rs']+dmpc['pair_margin'],'m'),
                 ('Hard floor forecast danh nghĩa',2*outer['rs']+dmpc.get('pair_hard_margin',dmpc['pair_margin']),'m')]
        parts+=['<details><summary><b>Thông số outer DMPC</b></summary>',
                table(['Thông số suy ra','Giá trị','Đơn vị'],derived),
                '<p>Horizon: '+html.escape(display(horizon_time))+' s. Reference và chu kỳ cập nhật khác vector field; không diễn giải so sánh này thành ưu thế thuật toán khi chưa có ablation tương ứng.</p>',
                table(['Tên trong cấu hình','Giá trị','Ý nghĩa'],[(k,v,dmpc_labels.get(k,'Tham số DMPC')) for k,v in dmpc.items()]),'</details>']
    parts+=['</section>']
    return ''.join(parts)


def saved_parameters(out):
    """For comparison report: use frozen inputs only, never follow config paths."""
    out=Path(out);directory=out/'inputs'
    required=[directory/'benchmark.json',directory/'base_config.json']
    if not all(p.exists() for p in required):
        return ''
    def read(path):return json.loads(path.read_text()) if path.exists() else None
    payload,cfg=map(read,required)
    initial=read(directory/'initial_state.json')
    settings=read(directory/'dmpc_config.json') or {}
    viewer=next((f'{name}/environment.html' for name in ('dmpc','vector_field') if (out/name/'environment.html').exists()),None)
    viewer_link=f'<p><a href="{viewer}">Mở môi trường và tube trong trình xem 3D</a></p>' if viewer else ''
    return viewer_link+render_parameters(payload,cfg,initial,settings.get('dmpc'))+(
        '<p>Nguồn snapshot: <a href="inputs/benchmark.json">benchmark.json</a> · '
        '<a href="inputs/base_config.json">base_config.json</a> · '
        '<a href="inputs/initial_state.json">initial_state.json</a> · '
        '<a href="inputs/dmpc_config.json">dmpc_config.json</a>.</p>')
