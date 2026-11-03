# DMPC v2: sửa lỗi và quy trình chạy từ nhỏ đến lớn

## Hai vấn đề đã sửa

### Optimizer và validator xét hai đối tượng khác nhau

v1 áp dụng separating planes tại prediction knots. Validator lại kiểm tra khoảng
cách nhỏ nhất của cả đoạn quỹ đạo tương đối giữa các knots. Do đó một nghiệm QP
có thể đạt residual nhưng vẫn bị validator từ chối; tăng iteration budget không
sửa được thiếu sót này.

v2 thêm collision cut tại endpoint và điểm gần nhau nhất bên trong đoạn của
nominal pair. SCP cập nhật lại các điểm cắt theo candidate. Validator vẫn kiểm tra
cả đoạn và không bỏ qua trường hợp khoảng cách nhỏ nhất nằm giữa hai knots.
Regression test dựng hai đường có endpoints cách nhau an toàn nhưng cắt nhau ở
giữa, xác nhận optimizer mới cũng loại candidate đó.

### Đệm dự báo bị coi nhầm là ngưỡng an toàn không thể phục hồi

v1 buộc mọi dự báo luôn >=10.5 m. Đây là 10 m an toàn + 0.5 m đệm. Khi feedback
thực lệch nhẹ khỏi plan, các plan cũ re-anchor có thể không còn thỏa 10.5 m mặc dù
khoảng cách thực vẫn trên 10 m. Các agent với neighbor plans cố định khó khôi phục
ngay phần đệm trong 0.1 s đầu; code kết thúc run như một lỗi safety feasibility.

v2 phân biệt:

- Ngưỡng đánh giá vật lý vẫn **10 m**, không sửa metrics.
- Forecast hard floor là **10.1 m** (thêm 0.1 m trên ngưỡng vật lý).
- Preferred forecast separation là **10.5 m**.
- Thêm một decision variable epsilon trong [0,0.4], cost 200 epsilon².
- Preferred separation tại thời gian dự báo t là
  `10.5 - epsilon * exp(-t/0.7)`.

Như vậy chỉ phần đệm có thể phục hồi dần; mức tối thiểu vẫn trên 10.1 m và phần
đệm trở lại gần 0.5 m ở cuối horizon. Không có slack cho phép bỏ ngưỡng an toàn
10 m. Validator kiểm tra hard floor độc lập với epsilon. Đây là thay đổi công
thức outer để sửa tính khả thi feedback, không phải giữ nguyên formulation v1.
Margins vẫn là lựa chọn thực nghiệm, không phải chứng minh robust safety.

Regression test bắt đầu hai UAV đứng yên cách 10.3 m: vẫn an toàn về vật lý, đủ
khoảng trống tube, nhưng thiếu preferred buffer. v2 tìm được lệnh hợp lệ và ghi rõ
buffer relaxation. Test hai UAV trùng vị trí vẫn bị từ chối, không được slack che lỗi.

Đối với lỗi dừng tại 50.42 s, log cũ cho thấy một số local QP không còn thỏa
residual, các plan dự phòng cũng không thỏa khoảng cách ưu tiên. Không xem đó
là chứng minh toàn bài toán va chạm 10 m vô nghiệm. Thay đổi v2 phân biệt rõ
physical infeasibility với việc chưa khôi phục được buffer.

## Những phần giữ nguyên

Tube, toàn bộ initial adaptive states, inner controller và gain, plant, nhiễu,
physical safety/speed/input thresholds, dt và quy tắc ra đích. Các kết quả v1 được
lưu riêng trong `artifacts/outer_comparison_before_fix/`.

## Chạy có cổng kiểm tra

```bash
python3 -m unittest discover -s tests -v
python3 -m drone_bench.cli.validate_dmpc
python3 -m drone_bench.cli.compare_outer --only dmpc \
  --validation artifacts/dmpc_validation/gates.json
```

Các gate mô phỏng: 1, 2, 4 UAV trong 3 s; 27 UAV trong 2 s. Gate yêu cầu không có
error hoặc violation thực; không yêu cầu hoàn thành cả tube trong thời gian ngắn.
Nếu gate thất bại hoặc fingerprint không khớp code/config hiện tại, lệnh compare
có `--validation` sẽ từ chối chạy. Gate chỉ là smoke/integration test, không thay
thế full run hay bảo đảm lý thuyết.

## Lặp lại đúng một chu kỳ lỗi

Runner lưu `failure_checkpoint.json` khi lỗi xảy ra ở chu kỳ outer: full inner
states, agent IDs, thời điểm và trạng thái nhân quả của DMPC trước lần giải.
Có thể kiểm tra một bản sửa ngay tại trạng thái lỗi, thay vì chạy lại từ đầu:

```bash
python3 -m drone_bench.cli.replay_dmpc_cycle \
  artifacts/outer_comparison/dmpc/failure_checkpoint.json
```

Không cần hay không được sửa inner để dùng checkpoint. Test serialize/restore
xác nhận cùng checkpoint tạo lại cùng lệnh. Việc replay một update thành công
vẫn cần được tiếp nối bằng các short gates và full regression.

## Kết quả sau sửa

Sau khi 28 test ban đầu và bốn short gates đạt, mới chạy lại đủ 27 UAV. Test khôi
phục checkpoint bổ sung cũng đạt (tổng 29 test).

- 27/27 UAV hoàn thành sau 72.5246 s; error=null, success=true.
- Min pair distance 10.3470 m; min local tube clearance 13.8755 m.
- Max actual speed 9.9315 m/s; không có violation theo metrics hiện tại.
- Buffer relaxation amplitude lớn nhất 0.1838 m, vẫn trong bound 0.4 m.
- Có 40 local QP candidate bị từ chối; các update cuối vẫn có plan được kiểm tra
  hợp lệ. Không đồng nghĩa mọi QP đều tối ưu/hội tụ.
- Mean wall time một outer update khoảng 1.002 s; p95 1.811 s, so với period 0.2 s.
  Còn 353/363 update trễ deadline trong thực thi tuần tự: chưa đạt real time.

Đối chiếu hash xác nhận inner/plant không thay đổi; invariants xác nhận cùng tube,
initial states và cấu hình vật lý. Gate fingerprint vẫn khớp mã control hiện hành.
Số liệu chỉ cho cấu hình/benchmark này; không phải chứng minh an toàn liên tục hay
bảo đảm cho mọi initial state/nhiễu khác.
