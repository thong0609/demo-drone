# Audit code và tuyên bố kết quả DMPC

Phạm vi: đọc implementation, đối chiếu artifacts, kiểm tra độc lập solver trên
các bài toán có nghiệm biết trước. Không thay controller, inner loop, plant,
config hay số liệu run; không chạy lại toàn bộ nhiệm vụ trong lượt audit này.

## Kết luận

Chưa phát hiện bằng chứng số liệu được bịa hoặc success được gán cứng. Code có
predictor, tối ưu local, trao đổi plan và receding-horizon execution thật. Nhưng
đây là **DMPC tự thiết kế cho benchmark**, không phải bản tái hiện nguyên gốc một
công trình DMPC/ADMM/DCBF. Các kết luận an toàn và ưu thế thuật toán phải được giới
hạn như các phát hiện sau. Không thể bảo đảm code không còn lỗi sau một audit.

## Phát hiện theo mức ảnh hưởng

### P1 — khoảng hở hướng kính không phải khoảng cách đến bề mặt tube

`Tube.project` và metric/controller dùng radius tại projection trừ khoảng cách
đến centerline. Nó không phải Euclidean signed distance đến bề mặt cong/thu bán
kính. Vì vậy diễn đạt “khoảng cách biên 13.88 m” không kèm chữ local/radial là thiếu
chính xác; kết quả hiện hành chỉ xác nhận **local radial clearance**.

Counterexample được tính giải tích: centerline (0,0,0)→(100,0,0), radius 5→55;
p=(50,25,0). API trả radial clearance=5 m, nhưng khoảng cách đến mặt bên hình nón
là 5/sqrt(1+0.5²)=4.4721 m. Kiểm tra radial>=5 không tương đương Euclidean surface
clearance>=5. Đây là thiếu sót nếu cần chứng nhận safety ball nằm trọn trong tube.
Counterexample không chứng minh run hiện tại đã vi phạm biên Euclidean; đại lượng
đó chưa được audit đầy đủ trên toàn run.

### P1 — phép so sánh chưa cô lập riêng tác dụng của DMPC

DMPC dùng cruise reference 9 m/s, vector field guidance bão hòa 11 m/s. Outer
period tương ứng là 0.2 và 0.02 s. Vì vậy không được lấy max speed thấp hơn để kết
luận DMPC xử lý ràng buộc tốt hơn chỉ nhờ cơ chế tối ưu. Cần thêm matched-reference/
matched-rate ablations và cùng ngân sách tuning trước kết luận nghiên cứu.

### P1 — success không bao gồm tính khả thi thời gian thực

summary.success=true nghĩa là mission hoàn thành và các physical metrics được lấy
mẫu đạt ngưỡng. Nó không tính deadline misses. Run lưu 353/363 updates quá hạn,
mean wall time 1.0024 s so với period 0.2 s. Plant mô phỏng chưa nhận trễ tính toán
thực này; không thể diễn giải thành controller triển khai được online ở 5 Hz.

### P2 — v2 thay đổi formulation, không chỉ sửa một lỗi lập trình

V1 yêu cầu forecast pair distance 10.5 m cứng. V2 dùng preferred buffer có slack
phạt, suy giảm theo horizon, với forecast floor 10.1 m. Khoảng cách vật lý chấm điểm
10 m vẫn giữ nguyên, nhưng formulation không giữ nguyên. Gọi hard preferred buffer
là “bug” một cách tuyệt đối là quá mạnh: đó là lựa chọn thiết kế có thể gây mất
khả thi dưới sai số mô hình. Phần bổ sung inter-knot cuts sửa sự không khớp giữa
optimizer và validator, nhưng chưa có thí nghiệm ablation tách riêng đóng góp hai
thay đổi. Không khẳng định cả hai đều là nguyên nhân duy nhất của lỗi 50.42 s.

### P2 — “không có vi phạm được đo” không phải bảo đảm liên tục

- Physical metrics lấy mẫu dt=0.02 s.
- Pair validation kiểm tra chính xác trên quỹ đạo nội suy tuyến tính giữa knots,
  không phải chính xác trên chuyển động exponential của predictor hoặc plant.
- Tube validation dùng knots và midpoint; không chứng nhận toàn bộ bề mặt/đoạn.
- Forecast hard floor 10.1 có tolerance mặc định 0.002 m: validator chấp nhận tới
  khoảng 10.098 m, không phải bound toán học đúng 10.100000 m.
- Predictor là cascaded lag có tau fit và bias estimator; không mô hình hóa đầy
  đủ adaptive inner states và không có bound robust đã chứng minh.

### P2 — không phải mọi QP đều tối ưu/hội tụ

Log có 9,148 optimal, 162 feasible_inexact, 40 residual_exceeded. Controller có thể
chấp nhận feasible_inexact khi primal và geometry hợp lệ; candidate residual vượt
ngưỡng bị loại, sau đó kiểm tra plan dự phòng. Vì vậy phát biểu “run không có lỗi
làm dừng” hợp lệ, nhưng “mọi QP đều hội tụ tối ưu” là sai.

Các trạng thái optimal là theo tolerance số của backend tự viết, chưa phải chứng
nhận từ solver độc lập trên tất cả 9,451 bài toán thực tế.

### P3 — vài quy ước cần giữ rõ khi mở rộng

- Input_saturation trong violation_steps hiện đo vượt input limit, không đo số
  lần actuator chạm bão hòa. Đừng dùng nó như saturation activation counter.
- Mỗi drone là một local optimization nhưng được giải tuần tự trên một máy,
  truyền thông giả định lý tưởng; chưa phải thực thi phân tán trên network.
- Miền dự báo sau goal được mở rộng thành cylinder theo endpoint tangent. Với
  tube hiện hành đã kiểm tra centerline chỉ qua terminal plane ở cuối; tube khác
  vẫn cần kiểm tra trước khi tái sử dụng quy ước ra đích này.

## Bằng chứng đã kiểm tra

Chạy lại bằng:

```bash
python3 -m experiments.audit_dmpc
```

Kết quả chi tiết: `artifacts/dmpc_audit/audit.json`.

- Tính lại 5 extrema từ CSV: khớp summary, gồm min pair 10.34697 m và max speed
  9.93154 m/s.
- Tính độc lập từ frames (thưa hơn CSV): min pair 10.34830, max speed 9.92940;
  chênh nhỏ đúng với việc decimate frames. Frame cuối active=[]; completion times
  đầy đủ 27 UAV.
- 101 mẫu của short gate 27 UAV khớp chính xác prefix full run.
- Source hashes của các file được manifest ghi nhận khớp source hiện tại.
- Centerline terminal-plane crossing duy nhất ở arc 639.19514 m, đúng cuối tube;
  không thấy dấu hiệu centerline tự kích hoạt điều kiện ra đích sớm trong map này.
- 40 QP 2D/3D so với liệt kê active sets độc lập: max solution error 3.04e-9.
- 12 QP 31 biến dựng từ nghiệm KKT biết trước: max solution error 8.41e-10.

Các kiểm tra này hỗ trợ tính nhất quán dữ liệu và backend, không loại trừ hết lỗi
ở horizon, geometry, coordination hoặc biến thể cấu hình chưa thử.

## Cách mô tả kết quả nên dùng

“Bản DMPC v2 tự triển khai đã đưa 27 UAV qua đích trên benchmark này; không phát
hiện vi phạm các chỉ số được lấy mẫu, trong đó biên tube là local radial clearance.
Thí nghiệm offline, chưa chứng minh an toàn liên tục, tính khả thi đệ quy hoặc ưu
thế so với baseline khi reference/update rate chưa được ghép tương ứng.”
