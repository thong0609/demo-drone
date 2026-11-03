# Kế hoạch: thay duy nhất outer loop bằng DMPC

Trạng thái: kế hoạch đã được triển khai và sửa thành DMPC v2. Xem
[DMPC_FIXES.md](DMPC_FIXES.md) và cấu hình hiện hành; các con số trong plan là
đề xuất ban đầu, không thay thế resolved_config của kết quả chạy.

## 1. Mục tiêu và phạm vi

So sánh enhanced vector field đang có (B0) với outer DMPC (B1). Giữ nguyên tube
`artifacts/paper_vb/benchmark.json`, mô hình vật lý, nhiễu, 27 UAV, inner loop,
tham số inner, thời gian tích phân 0.02 s, quy tắc ra đích và cách tính metrics.
Không chạy lại planner khi so sánh. Baseline hiện có hoàn thành nhưng vượt tốc;
không chỉnh nó để làm đẹp kết quả hoặc đổi ngưỡng thành công.

Các thay đổi được phép: mã outer mới; dữ liệu trạng thái truyền vào outer;
cơ chế giữ lệnh/chu kỳ giải bên trong outer; config riêng của DMPC; diagnostics
và báo cáo bổ sung, giữ nguyên định nghĩa các chỉ số cũ.

## 2. Ma trận thí nghiệm

| ID | Outer | Inner/plant/tube | Mục đích |
|---|---|---|---|
| B0 | PaperOuter | Cấu hình đã có | Baseline |
| B1 | DMPC vận tốc | Giống B0 | So sánh chính |

DMPC chưa dùng ADMM, DCBF hoặc một safety filter bổ sung. Chọn cơ chế phối hợp và
tránh va chạm rõ ràng bên trong DMPC thay vì coi tên DMPC đã xác định chúng.
Không gán B1 là bản tái hiện nguyên bản một bài optimal/dynamic tube khác.

## 3. Mốc 1 — đóng băng dữ liệu và kiểm tra baseline

- Lưu bản benchmark và toàn bộ initial_state của baseline thành inputs chung,
  kèm hash nội dung. Không chỉ kiểm tra hash scenario vì nó không bao hàm tube.
- Dùng cùng vị trí, vận tốc, bộ lọc, Lambda, e_hat, xi, trọng số và adaptive gains
  ban đầu. Đặc biệt runner hiện gọi inner.initial_state(..., vc) theo output outer:
  cần nạp state chung để tránh khởi tạo inner khác nhau giữa B0 và B1.
- Giữ lệnh khởi tạo chung trong bước đầu cho cả hai (lấy từ baseline), DMPC bắt
  đầu điều khiển từ lần cập nhật kế tiếp; không reset adaptive states khi đổi outer.
- Chạy B0 qua giao diện đã mở rộng; đối chiếu metrics, completion và trajectory
  với baseline trong tolerance số. Không đổi lệnh B0 ở các bước sau khởi tạo.

Đầu ra: input manifest và kết quả regression B0. Không triển khai DMPC trước khi
xác nhận adapter/runner không vô tình thay đổi baseline.

## 4. Mốc 2 — chuẩn bị giao diện và mô hình dự báo

Outer nhận thời gian, positions, velocities, filter_states và active_agents.
PaperOuter bỏ qua các dữ liệu bổ sung; vẫn tính lệnh như cũ. DMPC chỉ sử dụng
trạng thái hiện tại/thông tin trao đổi, không đọc nhiễu tương lai từ simulator.
Output vẫn là commands[id] = vector vận tốc 3D, kèm locations/clearance phục vụ
metrics. Geometry dùng chung được tách helper nếu cần, không thay định nghĩa.

Mô hình ứng viên cho state x=[p,v,zeta], input u=v_c:

    p_dot = v
    zeta_dot = (u-zeta)/tau_f
    v_dot = (zeta-v)/tau_v + b

Tau_f giữ giá trị bộ lọc hiện tại; tau_v là tham số mô hình dự báo. Fit/kiểm chứng
mô hình trên các tín hiệu vận tốc đổi biên độ/hướng bằng cùng inner/plant. b nếu
cần chỉ ước lượng từ dữ liệu quá khứ. Không thay rhs inner hoặc plant bằng mô hình
này. Phải đo sai số dự báo 1/2/3 giây trên dữ liệu validation độc lập và công bố
nó; mô hình không đủ chính xác thì cải tiến predictor, không sửa inner.

Chọn prediction margins theo sai số validation; đó là margin thực nghiệm, không
phải bound được chứng minh. Cố định mô hình và margins trước phép so sánh chính.

## 5. Mốc 3 — DMPC một UAV

Khởi đầu đề xuất: h=0.1 s; N=30 (3 giây); outer giải mỗi 5 bước plant; inner vẫn
chạy 0.02 s. Hold lệnh giữa lần cập nhật; dùng shifting/warm start. Chưa khẳng định
đạt real time trước khi đo solver latency.

Biến tối ưu: chuỗi 30 vector vận tốc tham chiếu và các state dự báo liên quan.
Hàm mục tiêu: tiến triển dọc tube; bám hướng guidance; giảm thay đổi lệnh; hạn chế
chi phí lệnh. Dùng guidance làm reference, không cộng thêm toàn bộ lực vector field
sau nghiệm DMPC. Không ép cứng mọi UAV đi đường tâm.

Ràng buộc: mô hình dự báo; norm lệnh <=11; norm vận tốc dự báo <=11; tube clearance
>=5 + margin; giới hạn thay đổi lệnh DMPC. Không đổi giới hạn thực trong metrics.
Near goal: reference tiếp tục qua mặt đích để tránh dừng trước đĩa cuối; nếu horizon
ra ngoài tube cần reference/miền thoát quy định rõ, chỉ bỏ constraints sau crossing
đĩa an toàn đã dự báo, không miễn constraints sớm vì gần đích.

Solver đề xuất: SCP với trust region, chuẩn Euclidean dạng conic khi phù hợp; tube
cong và tránh va chạm cần xấp xỉ cục bộ. Kiểm tra lại các ràng buộc hình học gốc sau
mỗi nghiệm, không coi tuyến tính hóa bất kỳ là chứng nhận tube containment. Chỉ
chấp nhận nghiệm đạt residual và sai số kiểm tra đã cấu hình. Một SOCP subproblem
không được gọi nhầm là QP. Chốt solver/phụ thuộc sau một prototype nhỏ; môi trường
hiện tại chưa có numpy/scipy/conic solver, cần kiểm tra khả năng cài hoặc cung cấp
backend trước khi hứa chạy đủ thí nghiệm tối ưu.

Đầu ra: 1 UAV qua đoạn thẳng, cong, co bán kính và ra đích; so predicted/actual.
Các fixture nhỏ chỉ phục vụ kiểm chứng; phép so sánh chính vẫn dùng nguyên 27 UAV.

## 6. Mốc 4 — phối hợp 2, 4 rồi 27 UAV

Mỗi UAV giữ prediction có timestamp. Bản đầu giả định truyền thông lý tưởng,
chia sẻ quỹ đạo với tất cả UAV đang hoạt động để tránh bỏ sót xung đột.

Trong một chu kỳ outer, tạo các plan đã dịch theo thời gian, cập nhật tuần tự kiểu
Gauss–Seidel với thứ tự ưu tiên luân phiên. Agent sau dùng plan mới nhất đã được
chấp nhận. Chạy số vòng phối hợp hữu hạn được cấu hình và ghi đầy đủ.

Tránh va chạm: tối thiểu 10 m + prediction margin. Có thể dùng nửa không gian phân
cách bảo thủ quanh nghiệm trước trong các bài toán con, sau đó kiểm tra lại khoảng
cách trên tập quỹ đạo cuối. Kiểm tra cả khoảng giữa các knot bằng subdivision/
đánh giá liên tục phù hợp; kiểm tra knot đơn thuần có thể bỏ sót va chạm.

Khởi tạo plans phải có feasibility check. Không xuất đồng thời các nghiệm độc lập
chưa kiểm tra ghép cặp. Việc giải các bài toán local tuần tự là mô phỏng cơ chế
phối hợp phân tán trên một máy, không phải distributed hardware execution.

Cohesion: ban đầu đưa vào cost mềm, nhưng ngưỡng 150 m trong metrics vẫn giữ
nguyên và mọi vi phạm vẫn tính thất bại. Công bố đây là lựa chọn formulation.

## 7. Mốc 5 — nghiệm không khả thi và diagnostics

Nghiệm cũ chỉ tái sử dụng nếu kiểm tra lại còn hợp lệ từ trạng thái thực. Cần kiểm
tra riêng cơ chế giảm tốc dự phòng; gửi u=0 không tương đương dừng tức thời và
không được xem là luôn an toàn. Nếu không có command/plan hợp lệ, ghi failure/
dừng run có lý do; không teleport, clip vận tốc hay sửa state để che vi phạm.

Thêm: solver status/residual, iterations, predicted minima, prediction errors,
coordination rounds, solve time và deadline misses. Đây là diagnostics bổ sung;
CSV/summary cũ vẫn giữ ý nghĩa và ngưỡng. Outer multi-rate không làm thay đổi dt plant.

## 8. Mốc 6 — phép so sánh chính

- Một paired run B0/B1 trên cùng frozen inputs để kiểm tra đường đi và lỗi.
- Freeze cấu hình DMPC sau validation, chạy lại đầy đủ và tạo báo cáo chung.
- So completion/time, min pair, min local boundary, max pair, actual speed, input
  và tracking RMSE bằng metrics chung. So wall/solver time và infeasibility riêng.
- Báo rõ outer update rate khác nhau; nếu muốn tách ảnh hưởng update rate, thêm
  một phép thử phụ B0 ở cùng rate, không thay thế baseline gốc.
- Không kết luận DMPC tốt hơn nếu chỉ nhanh hơn nhưng vi phạm safety nhiều hơn.
- Không hứa DMPC giải quyết vượt tốc: vận tốc dự báo bị giới hạn không đồng nghĩa
  plant thực có bảo đảm cứng khi predictor chỉ gần đúng.

Không tạo nhiều run lặp cùng seed nếu toàn hệ thống là deterministic rồi gọi đó
là thống kê. Thử nhiều mức nhiễu/trạng thái đầu là giai đoạn sau, chạy theo cặp cho
cả hai và phải được coi là mở rộng benchmark, không âm thầm thay điều kiện chính.

## 9. Tổ chức mã dự kiến

    algorithms/controllers/dmpc/
      predictor.py
      reference.py
      constraints.py
      optimizer.py
      coordination.py
      controller.py
    configs/experiments/paper_vb_dmpc.json
    cli/compare_outer.py
    tests/test_dmpc_*.py
    artifacts/outer_comparison/
      inputs/
      vector_field/
      dmpc/
      comparison.json
      report.html

Chỉ sửa interfaces/runner/CLI ở phần truyền dữ liệu, khởi tạo chung và chọn outer.
Giữ inner controller, dynamics và bộ lập tube ngoài phạm vi sửa. Metrics chỉ bổ
sung diagnostics, không đổi cách chấm cũ. Ưu tiên mốc 1–2 trước khi viết optimizer.
