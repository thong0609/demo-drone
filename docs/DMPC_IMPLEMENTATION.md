# Thực nghiệm outer DMPC trên tube tĩnh

**Cập nhật v2:** xem [DMPC_FIXES.md](DMPC_FIXES.md). Phần kết quả v1 ở cuối được
giữ làm lịch sử; kết quả hiện hành nằm trong `artifacts/outer_comparison/dmpc/summary.json`
và `artifacts/outer_comparison/comparison.json`.

## Chạy

```bash
python3 -m drone_bench.cli.compare_outer
```

Chạy riêng một nhánh:

```bash
python3 -m drone_bench.cli.compare_outer --only vector_field
python3 -m drone_bench.cli.compare_outer --only dmpc
```

Mở `artifacts/outer_comparison/report.html`. Hai nhánh nạp cùng tube và toàn bộ
adaptive state ban đầu từ baseline; inputs/manifest chứa hash chung. Kết quả từng
nhánh, CSV, replay và đồ thị nằm trong vector_field/ và dmpc/. Lệnh comparison
hoàn thành việc xuất báo cáo dù có run thất bại; xem summary.success và error,
không suy ra an toàn từ exit code của công cụ xuất báo cáo.

Chạy thử ngắn bằng `--duration 1 --output /tmp/dmpc-smoke`; không trộn kết quả này
với full run vì simulation hash khác. Các input/config đều được lưu trong outputs.

## Phạm vi thay đổi

Chỉ thêm outer DMPC và adapter truyền trạng thái. Inner controller, dynamics,
nhiễu, dt plant=0.02 s, tube, điều kiện ra đích và định nghĩa metrics giữ nguyên.
PaperOuter chỉ được bổ sung keyword time/states mà bỏ qua chúng. Runner có tùy chọn
nạp full state đã đóng băng; không gọi lại khởi tạo theo output riêng của DMPC.

DMPC nhận state đo, xuất vc. Trong bước đầu 0.02 s dùng cùng vc khởi tạo với baseline;
sau đó DMPC giữ lệnh giữa các chu kỳ cập nhật. Không clip vận tốc hay sửa plant.

## Formulation đã triển khai

- Prediction state [p,v,zeta], decision variables là 10 vector vc và một biến phục hồi buffer (31 biến).
- Horizon 30×0.1 s = 3 s; mỗi move block kéo dài 3 prediction steps. Shift controls
  theo đúng số prediction steps đã trôi qua; warm start là plan đã dịch.
- Outer update period **0.2 s**, điều chỉnh từ đề xuất ban đầu 0.1 s sau khi đo backend
  tuần tự. Plant/inner vẫn 0.02 s. Chưa đạt real time cho cả 27 UAV; deadline misses
  được ghi, wall-clock latency chưa đưa trở lại plant dưới dạng trễ truyền thông.
- Predictor cascaded first-order dùng ZOH giải tích. Tau_filter=.15 giữ nguyên;
  tau_velocity≈.2343 s fit bằng 12 s excitation riêng. Validation dùng profile khác,
  báo sai số 1/2/3 s. Bias được ước lượng nhân quả từ các lần đo trước; không đọc
  nhiễu tương lai hoặc state tương lai từ simulator. Bias limit là phần estimator,
  không phải việc clip plant.
- Cost quadratic: tracking vị trí .15, vận tốc 1, command .05, slew 1.5; thêm cohesion
  mềm .001 qua vị trí trung bình các plan lân cận. Reference tiến dọc tube theo
  cruise 9 m/s, giữ offset theo radius như Eq.22; không ép mọi UAV lên centerline.
- Giới hạn vc=11, vận tốc dự báo=10.5; preferred pair separation dự báo=10.5, hard floor=10.1 với slack buffer suy giảm theo horizon; tube clearance
  dự báo=6. Norm constraints dùng đa diện nội tiếp với 26 hướng (không thay bằng
  giới hạn từng trục rồi coi là tương đương norm Euclidean).
- Giới hạn thay đổi lệnh áp dụng giữa các move blocks bằng command_rate×block_time;
  không diễn giải nó là một bound cứng cho đạo hàm vc thực ở mọi lần replan.
- Tube convexification dùng 12 mặt trên tiết diện và slope bán kính quanh nominal
  plan; sau giải kiểm tra lại hình học gốc. Đây là xấp xỉ cục bộ, không chứng minh
  tube injective hay bảo đảm an toàn plant liên tục.
- Pair constraints dùng separating halfspaces khi khoảng cách nominal<24 m;
  nghiệm cuối vẫn được kiểm tra với tất cả UAV đang hoạt động. Kiểm tra khoảng cách
  nhỏ nhất của các đoạn quỹ đạo tương đối tuyến tính giữa prediction knots.
- Geometry kiểm tra tube ở knot và midpoint. Nó không chứng nhận toàn đường cong
  dự báo giữa knots, và càng không chứng nhận plant thực giữa các bước tích phân.
- Mỗi agent tối ưu khi cố định kế hoạch các agent khác. Tối đa ba vòng Gauss–Seidel,
  thứ tự ưu tiên luân phiên; dừng sớm khi tập plan cuối hợp lệ. Plan tạm chưa hợp lệ
  chỉ được giữ trong thương lượng, không gửi xuống plant.
- Chưa dùng ADMM, DCBF hoặc safety filter độc lập. Đây là thuật toán DMPC riêng của
  benchmark, không tái hiện nguyên bản một công trình khác.

## Solver và phụ thuộc

Không cần numpy/scipy. Backend C tự chứa giải QP lồi chặt bằng Hildreth dual
coordinate descent; Hessian nghịch đảo được chuẩn bị qua LU pivoting Python.
Các inverse-row products chỉ tính khi constraint được kích hoạt. SCP lặp tối đa
3 lần, QP tối đa 1500 sweeps. Phân biệt optimal / feasible_inexact /
residual_exceeded; status số và complementarity được lưu, không coi chỉ primal
feasible là đã tối ưu hoàn toàn. Không chạy solver nếu vẫn giữ lệnh giữa chu kỳ.

Cần Python 3 và gcc. Backend tự biên dịch vào `.cache/dmpc/` với hash source;
không tải mạng hoặc cài package toàn hệ thống. Native code còn tăng tốc phép chiếu
đường tâm và kiểm tra khoảng cách giữa quỹ đạo; kiểm thử đối chiếu Python geometry.

## Không khả thi và giới hạn

Candidate chỉ được nhận sau khi đạt kiểm tra residual và geometry. Khi candidate
bị từ chối, plan cũ phải được rollout lại từ trạng thái thực và kiểm tra với các
neighbors hiện tại. Nếu không hợp lệ, run kết thúc với error; không mặc định u=0
là an toàn và không trộn lại vector field để che lỗi DMPC. Fallbacks và thất bại
đều được ghi trong solver logs.

Margins là lựa chọn thực nghiệm, **không phải bound robust đã chứng minh**. Validation
predictor từng có max position error khoảng 1.35 m sau 3 s; không suy ra pair margin
0.5 m hay boundary margin 1 m bao phủ mọi sai số. Các violation thực vẫn được đo.

So sánh hiện tại dùng cruise reference 9 m/s cho DMPC, guidance bão hòa 11 m/s cho
baseline, đồng thời khác outer update period. Vì vậy đây là phép so sánh hai cấu
hình/kiến trúc trên cùng plant, chưa cô lập ảnh hưởng riêng của solver hay chứng
minh DMPC vượt trội. Một thí nghiệm tốc độ mục tiêu/update rate tương ứng sẽ là
ablation bổ sung, không được âm thầm thay baseline chính.

## Kiểm chứng

- Baseline qua giao diện mới: metrics/completion chính khớp kết quả lưu trước đó
  với sai khác 0; xem baseline_regression.json.
- QP có nghiệm biết trước, infeasibility, norm polytope; predictor ZOH so fine Euler;
  condensed sensitivity so rollout; between-knot collision; native geometry so Python.
- Integration: giữ lệnh giữa các lần update, không sửa state đầu vào, từ chối plan
  không khả thi. Toàn bộ suite chạy bằng `python3 -m unittest discover -s tests -v`.

Cấu hình có thẩm quyền là configs/experiments/paper_vb_dmpc.json. Các kết quả full
run có thẩm quyền là summary.json và comparison.json, không phải những con số
trong ghi chú khảo sát/plan ban đầu.

Lần đầu với giới hạn 150 QP sweeps dừng ở 25.42 s: candidate đã qua kiểm tra hình
học nhưng normalized primal residual khoảng 0.000307 vượt tolerance 0.0002. Không
kết luận bài toán vô nghiệm từ việc hết iteration budget. Trial này được giữ trong
`artifacts/outer_comparison/trials/qp150/`; tăng budget solver cho lần chạy chính,
không nới ngưỡng an toàn hoặc thay inner/plant.

Fixture v1 kiểm chứng riêng 1 UAV trên toàn tube với cùng inner/plant hoàn thành sau khoảng
69.25 s, max speed khoảng 8.89 m/s, không có violation theo bộ metrics hiện tại.
Dữ liệu nằm trong `validation/single_uav/`. Đây là fixture kiểm chứng thuật toán,
không thay thế kết quả chính của 27 UAV.

## Kết quả v1 — lưu lịch sử trước bản sửa

| Chỉ số | Vector field | DMPC |
|---|---:|---:|
| Hoàn thành | 27/27 | 0/27, dừng tại 50.42 s |
| Thời gian hoàn thành | 60.624 s | Không có |
| Khoảng cách cặp nhỏ nhất | 12.000 m | 10.491 m |
| Khoảng hở biên cục bộ nhỏ nhất | 8.796 m | 14.477 m |
| Khoảng cách cặp lớn nhất | 100.331 m | 64.560 m |
| Vận tốc thực lớn nhất | 12.523 m/s | 9.937 m/s |

DMPC không tìm được coupled plan hợp lệ sau ba vòng tại 50.42 s; không suy ra bài
toán gốc vô nghiệm. Trước lúc dừng, metrics thực không có violation. Đây là kết
quả trên **đoạn chạy chưa hoàn thành**, không phải chứng nhận an toàn toàn nhiệm vụ.
Cả hai summary.success đều false, nhưng vì lý do khác nhau: baseline vượt tốc;
DMPC chưa hoàn thành và hết ngân sách tìm plan hợp lệ. Không thể kết luận DMPC
thắng chỉ vì max speed thấp hơn hoặc clearance cao hơn trên một prefix ngắn hơn.

Các chu kỳ DMPC được chấp nhận có mean wall time khoảng 0.886 s, p95 1.980 s,
so với update period 0.2 s. Vì vậy hiện là thử nghiệm offline, chưa đạt real time.
Lần thương lượng thất bại cuối chưa được đo riêng trong thống kê cycle time.
`timing_and_prediction_summary.json` ghi phạm vi thống kê. Plan deviation sau
1/2/3 s còn bao gồm việc replan tiếp theo, không chỉ sai số predictor cùng control.

`invariants.json` xác nhận toàn bộ initial states, tube, inner parameters, plant,
simulation và các ngưỡng outer chung giống nhau. `baseline_regression.json` xác
nhận baseline qua adapter không đổi các số liệu chính. Các trial kỹ thuật trước
đó được giữ trong trials/, không coi là các run thống kê độc lập.

## Kết quả hiện hành — v2

Bản sửa đã hoàn thành 27 UAV sau 72.525 s, max speed 9.932 m/s, min pair 10.347 m,
min local boundary clearance 13.876 m, không có violation được đo. Xem
[DMPC_FIXES.md](DMPC_FIXES.md) để biết thay đổi preferred buffer, inter-knot cuts,
short gates và các giới hạn thực thi thời gian thực. Kết quả v1 phía trên chỉ là
lịch sử để đối chiếu lỗi cũ.
