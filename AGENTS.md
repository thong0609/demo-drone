# Chỉ dẫn và context nghiên cứu lâu dài

Đọc file này khi bắt đầu làm việc trong repo. Cập nhật khi người dùng chốt một
quyết định quan trọng; chi tiết lý thuyết, thực nghiệm và vấn đề mở nằm ở docs.
Thông tin dưới đây phản ánh trao đổi đến ngày 2026-11-10, không phải bảo đảm
kết quả sẽ giữ nguyên khi code/config thay đổi.

## Mục tiêu và phạm vi

- Bài báo chính: **Curve Virtual Tube Planning and Distributed Control for Swarm
  UAVs With Complex Constraints and Disturbances**, Yuan et al., IEEE TIE 2026,
  DOI 10.1109/TIE.2025.3603043. PDF nằm trong `/home/tts/Downloads/` với tên bắt
  đầu `1. Curve_Virtual_Tube_Planning...`. Đây là nguồn nền tảng của người dùng.
- Các mô phỏng tái hiện và DMPC trong repo là **thí nghiệm phụ**, không thay thế
  bài báo chính. DMPC tự thiết kế, chưa phải implementation ADMM/DCBF.
- Hướng hiện tại: **planner tube động realtime**, giả sử đã có thông tin vật
  cản. Chọn né toàn bộ vùng vật cản quét qua trong horizon trước.
- Outer/inner, perception và truyền thông thực để sau. Không tự gộp thiết kế
  lại các phần này vào một yêu cầu cải tiến planner.
- Theo dõi nghiên cứu theo cây trong ISSUES.md: **A — horizon** đang làm;
  **B — không gian–thời gian** chưa bắt đầu. Bốn bước tiếp theo nằm trong A:
  A1 chất lượng đường, A2 chọn/đánh giá horizon, A3 thời gian tính và kích hoạt,
  A4 đánh giá map/seed mới. Các phần dùng chung để sau nằm ở C.
- Điều chỉnh phạm vi mới nhất: A1 là hạn chế chất lượng của planner nền, tạm
  hoãn cải tiến tổng quát. Trọng tâm tiếp theo là A2 → A3 → A4. Giữ kiểm tra
  hình học và ghi nhận lỗi nền; không quy lỗi implementation thành lỗi bài báo.
- A3 hiện chỉ cần báo cáo điều kiện tính toán, giả thiết dự báo và hạn chế về
  độ trễ/tính hợp lệ của tube lúc dùng. Chưa yêu cầu scheduler có trễ, hard timeout
  hay bay thật. Không tự mở rộng nghiên cứu thành triển khai toàn hệ thống.

## Tài liệu cần tra đúng chủ đề

- [Mục lục](docs/README.md): bản đồ tài liệu.
- [Bài báo chính](docs/paper/CONTEXT.md) và
  [planner trong bài](docs/paper/TUBE_PLANNING.md).
- [Hiện thực/sửa công thức](docs/paper/FORMULA_FIXES.md): đọc khi cần đối chiếu
  code hiện hành với bài báo; đừng dùng mô tả cũ của geometry thay cho code mới.
- [Planner tube động](docs/dynamic_tube/PLANNER.md): lý thuyết, công thức,
  implementation, thực hành và kết quả hiện hành; đặc biệt phần 12 về cải tiến.
- [ISSUES](docs/dynamic_tube/ISSUES.md): quyết định, mã DT-xx và phần chưa xử lý.
- [Nhật ký trao đổi](docs/research/CONVERSATION_CONTEXT.md): lịch sử ban đầu,
  không phải trạng thái mới nhất.

## Các phát hiện không được quên

- Capsule hiện tại tương ứng cầu bán kính cố định, tâm chuyển động thẳng với
  vận tốc không đổi. Vùng quét bảo thủ có thể chứa start dù chưa va chạm hiện tại;
  phải ghi nhận thất bại, không bỏ constraint để tạo nghiệm.
- Đã triển khai lựa chọn balanced durations để giảm overshoot minimum snap,
  seed cố định, thử reuse waypoint và gate độ dài tăng sau làm trơn.
- Ca “xoáy” người dùng chụp là đường tâm nghiệm cuối bị uốn ngược sau làm trơn,
  không phải hoạt ảnh các iteration RRT*. Hai tube ở hai thời điểm là hai update.
- Balanced durations sửa được ca đã phát hiện và giảm chiều dài, nhưng max
  curvature lấy mẫu của ca đó tăng; một số case/seed khác vẫn có đoạn quay ngược.
  **Chưa giải quyết hoàn toàn hình dạng, độ cong hoặc injectivity** (DT-18).
- Đã có đánh giá exploratory 324 update với 9 cases, 3 seeds, 2 snapshots,
  6 modes và bộ kiểm tra horizon chung. Xem artifacts để lấy số liệu chính xác;
  chưa có held-out evaluation hoặc bảo đảm realtime toàn hệ thống (DT-19).
- Đã mở rộng A4 bằng 8 ca chuyển động theo nhiều hướng, thành 17 ca/612
  update và 306 replay. Report có đủ từng ca, mọi seed/phương án và lỗi; xem
  `artifacts/swept_volume_direction_evaluation/`. Mốc kiểm thử: 50 tests đạt.
  A4 vẫn chưa có tập map/seed held-out độc lập; không đánh dấu hoàn thành toàn bước.
- A2 có study riêng: `configs/experiments/swept_horizon_study.json`, lệnh
  `python3 -m drone_bench.cli.evaluate_horizon`, report
  `artifacts/swept_horizon_study/report.html`. Đã chạy 15 ca × 3 seed × 6 H ×
  2 mốc = 540 update/270 replay. Chỉ H đổi trong cùng case/seed. Không chốt H
  tối ưu từ bộ thử này; phân biệt đạt H, đạt đến update sau và check chuẩn 3 s.

## Ý tưởng mới: cầu bao vật thể bất kỳ

- Người dùng đề xuất thay vật thể không rõ chủng loại/hình dạng bằng một cầu
  bao để dùng chung planner hình cầu. Đây là **hướng mở rộng đang thảo luận**,
  chưa có backend perception/tính cầu bao cho vật thể bất kỳ.
- Cầu phải bao toàn bộ vật thể, có xét sai số. Bao point cloud quan sát được
  chưa chứng minh đã bao phần bị che khuất.
- Cầu bao tổng quát hóa **hình học**, không tự giải quyết chuyển động tương lai.
  Vẫn cần vị trí, kích thước, timestamp, vận tốc và giả thiết/bound dự báo.
- Cầu quá lớn cho vật dài/mảnh có thể làm planner từ chối một đường thực ra đi
  được. Vật đổi hướng/gia tốc hoặc bán kính thay đổi cần vùng quét khác capsule
  bán kính cố định hiện tại. Theo dõi tại DT-20.

## Cách làm việc người dùng mong muốn

- Trao đổi bằng tiếng Việt. Công thức dài/lý thuyết ghi vào Markdown, cung cấp
  link; terminal hiển thị công thức không thuận tiện cho người dùng.
- Nói đơn giản, câu ngắn, kết quả chính trước. Khi lên kế hoạch: làm gì, vì sao,
  xong có gì để xem. Khi báo cáo: đã sửa gì, kết quả ra sao, còn lỗi gì. Giải
  thích thuật ngữ khi dùng, không dồn mã issue và nhiều con số vào câu trả lời.
- Giữ tài liệu gọn: cập nhật PLANNER.md và ISSUES.md hiện có, không tạo thêm
  file plan cho mỗi cuộc trao đổi. Phân biệt đã chốt, đề xuất, đã làm và đã đo.
- Khi thống nhất bước làm tiếp, ghi ngay dưới đúng hướng/bước cha trong
  ISSUES.md, có trạng thái và kết quả cần có để hoàn thành. Giữ việc đã làm và
  việc chưa làm trong cùng bước; không thêm một danh sách issue ngang hàng rời rạc.
- Replay: vật cản chạy theo thời gian; chỉ một tube. Giữ tube tới update tiếp
  theo rồi xóa/thay tube mới. Không thêm walkthrough, chồng tube cũ hay giả lập
  nội suy chuyển tiếp chưa được kiểm chứng.
- Khi sửa replay, kiểm tra đúng report mặc định bằng nút Chạy thực tế trong
  browser, qua các mốc update; chỉ gọi seek hoặc kiểm tra dữ liệu chưa đủ.
- Không diễn giải chứng nhận hình học tube thành an toàn của đàn thực, độ cong
  lấy mẫu thành bound liên tục, hay số đo solver thành end-to-end realtime.
- Giữ baseline/config/artifacts có provenance; không clip state, sửa metrics
  hoặc bỏ các run thất bại để làm đẹp kết quả. Khi cập nhật docs, sửa liên kết.

## Lệnh thường dùng

```bash
python3 -m unittest discover -s tests -v
python3 -m drone_bench.cli.plan_swept_volume
python3 -m drone_bench.cli.evaluate_swept_volume
```

Report mặc định: `artifacts/swept_volume/report.html`. Đánh giá mở rộng hiện hành:
`artifacts/swept_volume_direction_evaluation/report.html`; bộ 9 ca lịch sử giữ ở
`artifacts/swept_volume_evaluation/report.html`. Lệnh replay không tham số dùng
`configs/experiments/swept_volume_replay.json`; config `swept_volume.json` giữ
fixture baseline. Xem docs trước khi đổi giao thức hoặc so kết quả khác config.
