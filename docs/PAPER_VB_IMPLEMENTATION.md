# Thí nghiệm 2: mô phỏng 3D mục V-B

**Cập nhật 2026-11-10:** cấu hình/geometry hiện tại đã được sửa theo audit ở
[PAPER_FORMULA_FIXES.md](PAPER_FORMULA_FIXES.md). Các mô tả polyline, raw RBF và
Delta phụ thuộc p bên dưới ghi lại phiên bản tạo artifacts lịch sử. Output mặc
định mới là `artifacts/paper_vb_continuous`; không dùng các kết quả cũ bên dưới
như số liệu của code đã sửa.

## Chạy và xem kết quả

Từ thư mục gốc, không cần cài thư viện ngoài Python 3:

```bash
python3 -m drone_bench.cli.run_paper_vb
```

Mở `artifacts/paper_vb/report.html`: đồ thị, ảnh tại 0/25/50 s và liên kết replay
3D có nút chạy/dừng, tua thời gian, xoay và zoom. Đồ thị và ảnh là SVG độc lập,
có thể đưa vào báo cáo. Replay mở offline, không cần server.

CLI trả mã 0 khi hoàn thành và tất cả tiêu chí được đo đều đạt. Mã 2 có nghĩa
bản mô phỏng đã xuất kết quả nhưng chưa đạt ít nhất một tiêu chí (hoặc có lỗi
trong quá trình tích phân); xem `summary.json`. Không hiểu mã 2 là thiếu kết quả.

Nạp một tube cố định để so sánh bộ điều khiển:

```bash
python3 -m drone_bench.cli.run_paper_vb \
  --benchmark artifacts/paper_vb/benchmark.json \
  --output artifacts/paper_vb_reuse
```

Kiểm tra độ nhạy bước thời gian trên **cùng tube**:

```bash
python3 -m drone_bench.cli.run_paper_vb \
  --benchmark artifacts/paper_vb/benchmark.json \
  --dt 0.01 --output artifacts/paper_vb_dt001
```

Chỉ lập kế hoạch: thêm `--plan-only`. Đổi cấu hình bằng `--config`; đổi thời gian
chạy bằng `--duration`. `--dt` và `--duration` được lưu vào resolved_config.

## Công thức và module

| Thành phần | Bài báo | Module |
|---|---|---|
| SDF khoảng cách đến vật cản/biên world | III | `environment/world.py`, `obstacles.py` |
| Enhanced RRT*, chi phí hỗn hợp, Visual | Algorithms 2–3, (15)–(16) | `algorithms/planners/enhanced_rrt.py` |
| Minimum snap + tracking, ràng buộc vị trí/đạo hàm/corridor | (17)–(21) | `algorithms/planners/minimum_snap.py` |
| Ghép pipeline và tạo bán kính | Algorithm 1 | `algorithms/planners/paper_tube.py` |
| Guidance riêng theo offset toàn cục | (22)–(24) | `algorithms/controllers/paper_outer.py` |
| Thế đẩy/hút và đạo hàm giải tích | (25)–(33), (35) | `algorithms/controllers/potentials.py` |
| Vận tốc vòng ngoài và norm saturation | (2)–(3), (34) | `algorithms/controllers/paper_outer.py` |
| Dynamics tịnh tiến | (1) | `simulation/dynamics.py` |
| Filter, error, NN, SPEM, actor–critic | (41)–(56) | `algorithms/controllers/paper_inner.py` |
| LDO | (57) | `algorithms/controllers/paper_inner.py` |
| Hard saturation, tanh proxy và hệ phụ | (4)–(5), (60) | `algorithms/controllers/paper_inner.py` |
| Điều khiển trong, cập nhật robust gain | (61), đoạn sau (66) | `algorithms/controllers/paper_inner.py` |
| Chạy và đo tiêu chí chung | V-B, Fig. 8 | `simulation/runner.py`, `metrics.py` |

SDF tính giải tích theo primitive, không dùng voxel grid. QP dùng đa thức Bernstein
bậc 7, tham số cục bộ chuẩn hóa và thời lượng segment; snap integral và tracking
integral được tích phân giải tích. LU pivoting giải hệ KKT, active set xử lý bất đẳng
thức. Bài báo dùng interior-point; solver khác nhưng cùng dạng mục tiêu/ràng buộc.
Corridor được siết qua convex hull control points, bảo đảm cho toàn bộ đa thức
(thay vì chỉ lấy mẫu). Kiểm tra residual equality và inequality trước khi chấp nhận.

Bán kính từ khoảng hở SDF tại các waypoint, sau đó minimum-snap. Bước bổ sung **co
đồng đều bán kính sau QP** nhằm bảo đảm khoảng hở tube với world/vật cản bằng chứng
nhận swept-ball hiện có. Bước này thay đổi các giá trị bán kính waypoint, vẫn giữ
tính trơn và được lưu hệ số scale; đây là biện pháp kiểm tra an toàn triển khai,
không được coi là công thức có sẵn trong bài báo. Tube xuất ra là bản lấy mẫu
piecewise-linear của đa thức, dùng chung cho các controller.

Guidance đạo hàm theo (22), không đổi sang parallel-transport offset. Nếu k1*lp >= vm,
độ lớn guidance chính xác bằng vm với mọi chiều dài còn lại không âm; nhờ vậy không
cần tích phân chiều dài mỗi bước. Nếu không, code tích phân độ dài trên các đoạn mẫu.
Set Li theo (9) chỉ chứa các cặp ls <= d <= lm. Khi d > lm, code ghi vi phạm thay vì
âm thầm thay luật điều khiển.

## Tham số từ bài và các giả định

`configs/experiments/paper_vb.json` phân biệt danh sách paper_parameters và assumptions.
Giữ 27 UAV; vm=11; rs=5; ra=10; ls=100; lm=150; P/C/K/Gamma/eta theo V-B.
Kích thước world, điểm đầu–đích và bán kính tube tối đa theo V-B.

Các giá trị **tự chọn để có thể chạy lại**, không phải thông số tác giả đã cung cấp:

- Tọa độ/kích thước vật cản, seed 17 và thông số enhanced RRT*.
- Corridor, gamma minimum-snap, tham số lực k1–k4, delta, lp.
- Khối lượng 1 kg, drag, gravity, giới hạn đầu vào từng trục 30 N.
- Bất định Delta_k = 0.15 sin(v_k/4 + p_k/80).
- Nhiễu d_k = 0.25 sin((0.45+0.1k)t + 0.37i + k), k=0,1,2.
- NN RBF: Z=[v,zeta,zeta_dot] được scale theo cấu hình, bias và 3 tâm nằm trên
  đường chéo không gian 9D; actor/critic khởi tạo trọng số 0.
- Bộ lọc tau=0.15; Ka=2; Q=10I; eta3=1; robust smoothing=0.5.
- 3 tiết diện ban đầu, mỗi tiết diện 3×3 UAV; vị trí và toàn bộ adaptive states
  ban đầu được xuất. Vận tốc thực ban đầu 0; zeta(0)=vc(0) đúng (41);
  e_hat(0)=e(0), xi(0)=-Ce(0), D_hat(0)=0.
- Dynamics dùng uc=hard_sat(H) theo (4); u_bar=tanh_proxy(H) theo (5) được giữ
  riêng cho hệ phụ (60). Do đó có sai khác Delta_uc như (59). Không thay uc bằng
  proxy trong plant, không clip vị trí hoặc vận tốc.
- Explicit midpoint cho toàn bộ state vòng trong; outer giữ lệnh trong một dt.
- UAV đi qua đĩa cuối an toàn thì hoàn thành và rời miền mô phỏng, không đỗ chặn
  lối ra. Chỉ đo UAV đang bay; thời điểm crossing được nội suy trong một bước.

## Dữ liệu đầu ra

- `benchmark.json`: world và tube cố định.
- `planning.json`: waypoints, chi phí, hệ số đa thức, residual QP, scale bán kính.
- `manifest.json`: hash benchmark chuẩn hóa, seed, cấu hình và provenance.
- `initial_state.json`: vị trí và trạng thái đầy đủ của 27 UAV.
- `metrics.csv`: đo mỗi bước dt, không chỉ mỗi frame hoạt ảnh.
- `trajectory.json`: frame vị trí, vận tốc và ước lượng nhiễu, cùng summary.
- `summary.json`: số UAV hoàn thành, thời gian, extrema, số bước vi phạm từng loại.
- `figures/`: SVG đồ thị và ảnh chụp; `report.html`, `replay.html`, `environment.html`.

Mỗi luật điều khiển có module riêng. `simulation.runner.run` nhận outer và inner
qua tham số; có thể thay thuật toán mà giữ dynamics, cấu hình ban đầu và cách đo.
Xem `simulation/interfaces.py` cho hợp đồng state và diagnostics tối thiểu.

## Phạm vi kết luận

Đây là hiện thực thuật toán và thí nghiệm trên bản đồ chung tự dựng, không phải
bản tái tạo số liệu chính xác của tác giả. Không dùng độ dài/chi phí ở đây để tuyên
bố hơn/kém số liệu bài báo trên một map khác.

Khoảng cách biên trong controller/đồ thị là **khoảng hở hướng kính tại phép chiếu
đường tâm cục bộ**, không phải khoảng cách Euclidean chính xác đến toàn bộ bề mặt
cong. Việc tìm tiết diện dùng neighborhood tracking; chưa chứng minh tính duy nhất
của tiết diện hay tính injective của tube. `min_obstacle_clearance` được đo độc lập
bằng world SDF. Các bất đẳng thức được đo theo thời gian rời rạc, không phải chứng
minh an toàn liên tục. Bài báo chứng minh Vo không tăng trong mô hình vận tốc ảo;
Ve chỉ có bound dạng (76). Không diễn giải chúng thành bảo đảm cứng cho toàn bộ
plant số có sai số bám.

Lần chạy baseline dt=0.02 và kiểm tra dt=0.01 đều đưa 27 UAV qua đích, giữ các ngưỡng
khoảng cách đã đo, nhưng **vận tốc thực vượt 11 m/s** (cỡ 12.4–12.5 m/s).
`success=false` phản ánh điều đó. Giảm dt vẫn còn vượt tốc; chưa đủ cơ sở kết luận
là lỗi riêng của bước tích phân. Không cắt vận tốc sau tích phân để làm đẹp kết quả.
Các tham số chưa công bố và xấp xỉ hình học ở trên giới hạn khả năng đối chiếu trực
tiếp; kết quả không chứng minh bài báo sai hoặc đã tái hiện đầy đủ bảo đảm của nó.

Đồ thị `disturbance_rmse` đo sai số LDO so với residual của mô hình hiện tại
`Delta - zeta_dot - estimated_actor + d`. Nó bao gồm sai số trọng số NN hiện tại;
không phải sai số với D=epsilon*+d dùng W* tối ưu chưa biết trong chứng minh (48).
