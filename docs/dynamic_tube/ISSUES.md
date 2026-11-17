# Tiến độ nghiên cứu planner tube động

Cập nhật theo trao đổi ngày 2026-11-10. Đây là nơi theo dõi việc đã làm và việc
còn lại. Lý thuyết, công thức và kết quả chi tiết nằm trong [PLANNER.md](PLANNER.md).

## Nhìn nhanh: hai hướng nghiên cứu lớn

| Hướng | Cách làm | Trạng thái |
|---|---|---|
| **A. Né vùng quét trong horizon** | Cấm toàn bộ vùng vật cản sẽ đi qua trong một khoảng nhìn trước, rồi tìm tube tránh vùng đó | **Đang thực hiện**: có bản chạy và đánh giá ban đầu |
| **B. Lập kế hoạch không gian–thời gian** | Xét cả vị trí và thời điểm đi qua; có thể chọn đi trước, đi sau hoặc chờ vật cản | **Chưa bắt đầu** |

**Việc chính tiếp theo: A2 — đánh giá và chọn horizon**, rồi A3 và A4.
A1 giữ để theo dõi hạn chế của planner nền; không yêu cầu giải quyết toàn bộ
độ cong/đường vòng để tiếp tục nghiên cứu vùng quét.
Không coi hướng A hoàn thành chỉ vì demo đã chạy; không tự chuyển sang hướng B
khi chưa có yêu cầu.

Quy ước: `[x]` đã làm, có kết quả hoặc bằng chứng; `[ ]` chưa xong. Một bước có
thể đã làm một phần nhưng vẫn chưa hoàn thành. Khi đánh dấu xong, ghi nơi xem
kết quả. Không tạo file plan mới cho mỗi bước nhỏ.

## A. Hướng 1 — Né vùng vật cản quét qua trong horizon

Phạm vi hiện tại: biết trước thông tin vật cản; vật cản cầu, kích thước cố định,
vận tốc không đổi. Giữ outer/inner; nghiên cứu planner hình học trước. Chưa có
chuyển tiếp tube cho đàn đang bay, perception hoặc truyền thông thực.

### A0. Xây bản thử đầu tiên — Đã làm trong phạm vi mô hình đã chọn

- [x] Định nghĩa vị trí, bán kính, vector vận tốc, hệ tọa độ, đơn vị và timestamp.
- [x] Kiểm tra tuổi dữ liệu, từ chối quan sát ở tương lai/quá cũ và ngoại suy tới
  thời điểm tính. CLI dùng quan sát hiện tại; không đọc tương lai thật để dự báo.
- [x] Tạo capsule bao toàn bộ chuyển động liên tục trong horizon; có đệm dự báo.
- [x] Kết hợp capsule với vật cản tĩnh và biên world để truy vấn khoảng hở.
- [x] Dùng enhanced RRT* + minimum snap lập lại toàn tuyến với hai đầu cố định.
- [x] Kiểm tra cả thể tích tube, giới hạn bán kính và đường đa thức liên tục;
  không chỉ kiểm tra đường tâm hoặc vài ảnh chụp vật cản.
- [x] Kiểm tra lại tube cũ trước khi giữ; nếu thất bại thì báo rõ và trả tube=null.
- [x] Lưu dữ liệu, cấu hình, hash source, trạng thái và thời gian mỗi lần tính.
- [x] Có replay chạy theo thời gian, xóa tube cũ khi thay tube mới, không vẽ chồng.
- [x] Chạy kiểm thử và bộ đối chứng ban đầu: 48 tests đạt; 324 lượt đánh giá
  thuộc 9 tình huống × 3 seed × 2 thời điểm × 6 phương án.

**Xem kết quả:** `artifacts/swept_volume/report.html`,
`artifacts/swept_volume_evaluation/`, [PLANNER.md, phần 12](PLANNER.md#12-cải-thiện-hình-dạng-và-đánh-giá-đối-chứng).
Các số test/lượt đánh giá là mốc đã chạy, cần cập nhật khi có kết quả mới.

### A1. Chất lượng đường của planner nền — Ghi nhận hạn chế, tạm để sau

**Mục tiêu:** đường tránh vật cản nhưng không vòng bất thường hoặc cong quá gắt.

**Ranh giới phạm vi:** nghiên cứu chính là tạo vùng cấm từ chuyển động vật cản
trong horizon và cập nhật tube bằng planner nền. Tối ưu tổng quát enhanced RRT*
hoặc minimum snap không phải đóng góp bắt buộc của nghiên cứu này. Các thử sửa
A1 đã làm được giữ như biến thể/ablation, không tự tiếp tục mở rộng A1.

Đoạn vòng quan sát được thuộc bản hiện thực và cấu hình đang chạy; chưa đủ bằng
chứng quy thành lỗi thuật toán hoặc kết quả của tác giả bài báo. Dùng thuật toán
cùng tên không có nghĩa đã dùng đúng mọi tham số/solver của tác giả.

Vẫn phải kiểm tra tránh vật cản, bán kính hợp lệ và ghi nhận candidate bị từ
chối. Nếu planner nền làm thí nghiệm không thể đánh giá được, chỉ xử lý phần
tối thiểu cần thiết và công bố rõ, không âm thầm đổi planner hoặc bỏ run xấu.

Đã làm:

- [x] Tái hiện ảnh “xoáy”: waypoint tiến dần nhưng đường sau minimum snap có
  đoạn quay ngược. Đây là nghiệm cuối của một update, không phải hoạt ảnh RRT*.
- [x] Thử đúng cùng waypoint với thời lượng các đoạn bằng nhau. Ca đã phát hiện
  giảm chiều dài khoảng 519.4 → 428.8 m và bỏ đoạn quay ngược theo hướng start–goal.
- [x] Đo chiều dài, phần tăng sau làm trơn, độ cong lấy mẫu và quãng đi ngược.
- [x] Thêm ngưỡng phần tăng chiều dài sau làm trơn cho cả tube mới và tube giữ lại.
- [x] Thử seed cố định và làm trơn lại waypoint cũ; candidate vẫn phải kiểm chứng.
- [x] Có kiểm thử hồi quy cho ca vòng và cho tái sử dụng waypoint.

Các việc còn mở, tạm hoãn ngoài trọng tâm nghiên cứu:

- [ ] Xử lý đánh đổi: ở ca vừa sửa, độ cong cực đại lấy mẫu tăng khoảng
  0.267 → 2.351 1/m dù đường ngắn hơn.
- [ ] Chọn cách hạn chế độ cong/overshoot phù hợp; đánh giá riêng từng thay đổi.
- [ ] Kiểm tra nguy cơ tube tự chồng lấn, kể cả khi tâm không chạm vật cản.
- [ ] Kiểm tra lại các seed/case vẫn có đoạn quay ngược. Không cấm mọi đoạn đi
  ngược X vì tuyến hợp lệ ở map khác có thể cần như vậy.
- [ ] Xác định tiêu chí chất lượng và chỉ rõ phần nào được chứng nhận, phần nào
  mới lấy mẫu; không dùng replay nhìn đẹp thay cho kiểm tra hình học.

**Xong khi:** có tiêu chí rõ, có đối chứng cùng dữ liệu, giảm lỗi hình dạng mà
vẫn giữ khoảng hở; công bố các ca không đạt. Nếu chưa chứng nhận độ cong hoặc
không tự chồng lấn thì phải giữ rõ giới hạn, không đánh dấu đã giải quyết hết.

**Sản phẩm cần xem:** replay ca trước/sau sửa, bảng chiều dài/độ cong/khoảng hở,
kiểm thử hồi quy và cập nhật PLANNER.md. Các chi tiết cũ tương ứng DT-18.
Tiêu chí trên dùng khi quay lại A1; không phải cổng bắt buộc trước A2.

### A2. Đánh giá và chọn horizon — Đã có bộ so sánh, chưa chốt quy tắc chọn H

**Mục tiêu:** hiểu nhìn trước bao lâu là hữu ích và khi nào vùng cấm quá bảo thủ.

Đã làm:

- [x] Có thử horizon 1, 3 và 6 giây trong các bộ thử.
- [x] Chấm mọi phương án bằng cùng bộ kiểm tra tương lai của từng tình huống,
  kể cả phương án chỉ né vị trí hiện tại và tube tĩnh.
- [x] Giữ các ca vùng quét chứa start trong kết quả, không bỏ để tăng tỷ lệ đạt.
- [x] Có file cấu hình dải H và lệnh `evaluate_horizon`; report cho chọn H để
  đổi replay và so cùng case/seed. Lý thuyết/cách chạy ở PLANNER.md, phần 14.
- [x] Đã chạy bộ 15 case × 3 seed × 6 H × 2 snapshot = 540 update và 270 replay;
  kiểm tra dữ liệu/config tương ứng và source hashes. Kết quả ở
  `artifacts/swept_horizon_study/report.html`, PLANNER.md phần 14.
- [x] Kiểm tra trên Chrome: đổi H ngắn/dài tải đúng replay, dòng so sánh đổi
  theo và nút Chạy trong replay nhúng thực sự đi qua mốc cập nhật. 53 tests đạt.

Cần làm tiếp:

- [x] Thiết kế phép thử chỉ thay horizon, giữ nguyên map, vật cản, seed, budget
  tính toán và cách đo; có test xác nhận config chỉ khác H trong cùng cặp.
- [x] Khảo sát H=0.5/1/2/3/5/6 trên 15 ca gồm kích thước, hướng và tốc độ khác nhau.
- [x] Đo tỷ lệ đạt, certificate, chiều dài, bán kính, mức đổi tuyến và thời gian;
  tách kiểm tra trong H, tới lần cập nhật sau và khoảng nhìn trước chuẩn 3 s.
- [ ] Phân loại ca thất bại: hết đường hình học, quá bảo thủ, hết budget tìm kiếm,
  lỗi solver hoặc không chứng nhận được. Không gọi mọi thất bại là vô nghiệm.
- [ ] Phân biệt “không có đường trong vùng quét bảo thủ” với “vẫn đi được nếu
  chọn thời điểm phù hợp”. Nếu cần, dùng một đối chứng thời gian đơn giản;
  việc đó chưa đồng nghĩa đã triển khai đầy đủ hướng B.
- [ ] Chọn hoặc đề xuất quy tắc chọn horizon và đệm dự báo, ghi rõ phạm vi áp dụng.

**Xong khi:** có bảng/đồ thị so horizon trong cùng điều kiện, giải thích được
đánh đổi và các ca thất bại; chưa cần chứng minh có một horizon tốt cho mọi map.

**Sản phẩm cần xem:** bộ config cố định, kết quả khảo sát horizon và khuyến nghị
có bằng chứng trong PLANNER.md. Liên quan các mục cũ DT-02/03/11/12/19.

### A3. Điều kiện tính toán và giới hạn khi dùng tube — Thu hẹp phạm vi hiện tại

**Mục tiêu hiện tại:** công bố điều kiện thử nghiệm, cách dự báo chuyển động và
phạm vi đo thời gian; nêu rõ độ trễ và tính hợp lệ lúc dùng tube là phần chưa
giải quyết. Chưa triển khai scheduler có trễ, hard timeout hoặc thử nghiệm UAV
thực chỉ để hoàn thành nghiên cứu planner hiện tại.

Đã làm:

- [x] Đo thời gian update, ghi deadline miss, status và error.
- [x] Lưu thời điểm quan sát và khoảng hiệu lực của snapshot.
- [x] Chứng nhận lại tube trước khi giữ; không mặc định tube cũ hay lệnh dừng là an toàn.
- [x] Có phép so thời gian trên cùng budget trong bộ thử ban đầu.

Việc cần hoàn thiện trong báo cáo hiện tại:

- [x] Ghi rõ tính từ quan sát hiện tại, dự báo vận tốc không đổi, tạo vùng quét
  rồi dùng enhanced RRT* + minimum snap; không biết chắc tương lai ngoài thực tế.
- [x] Ghi rõ thời gian update đo được chưa gồm nhận thức, truyền thông và toàn
  bộ chuẩn bị dữ liệu; replay chưa đưa trễ tính toán vào thời điểm thay tube.
- [ ] Rà lại bảng thông số thử nghiệm: map, vật cản, H, lịch cập nhật, planner,
  phiên bản code, cấu hình máy/tài nguyên thực sự dùng; không suy đoán hardware
  không được ghi nhận trong lần chạy.
- [ ] Hoàn thiện mục hạn chế và kết luận: chứng nhận hình học theo dự báo không
  bảo đảm candidate còn an toàn tại thời điểm triển khai lên UAV.

Các việc triển khai nâng cao để sau, không phải điều kiện bắt buộc của bản báo
cáo planner hiện tại:

- [ ] Chốt chu kỳ cập nhật, deadline và phần cứng dùng để đánh giá.
- [ ] Đo đủ thời gian chuẩn bị dữ liệu, dự báo, dựng vùng quét, tìm đường,
  làm trơn và kiểm chứng; báo mean/p95/p99/max và tỷ lệ quá hạn.
- [ ] Xét vật cản tiếp tục chuyển động trong thời gian tính; định nghĩa thời
  điểm candidate có thể được dùng và kiểm tra lại khi bàn giao.
- [ ] Xử lý snapshot hết hạn trước khi có kết quả mới; không kéo dài hiệu lực âm thầm.
- [ ] Có giới hạn thời gian tính cứng hoặc cơ chế hủy/kết thúc phù hợp.
- [ ] Định nghĩa trạng thái planner trả về khi không kịp/không có tube hợp lệ.
  Hành vi dừng/né của UAV thật vẫn thuộc phần controller để sau.

**Xong phần báo cáo khi:** thông số và phạm vi đo rõ, giả thiết/nhược điểm đầy
đủ, không tuyên bố đã có realtime hoặc an toàn toàn hệ thống. Không đánh dấu
các việc triển khai nâng cao là đã xong chỉ vì đã nêu chúng trong hạn chế.

**Sản phẩm cần xem hiện tại:** bảng thông số, thời gian đo và mục hạn chế trong
PLANNER.md. Lịch kích hoạt, kiểm tra lại tube sau trễ và thử nghiệm thực để sau.
Liên quan các mục cũ DT-08/10/19.

### A4. Kiểm tra trên bản đồ và dữ liệu mới — Đã có bộ nhỏ, cần mở rộng

**Mục tiêu:** kiểm tra phương pháp ngoài những ca đã dùng để phát hiện và sửa lỗi.

Đã làm:

- [x] Có tình huống vật cản lớn/nhanh/nhiều vật, vật cản tĩnh và ca start bị chặn.
- [x] Bổ sung 8 ca chuyển động: hai hướng cắt ngang, hai hướng dọc tuyến,
  trên xuống/dưới lên, chéo và ba vật cản hội tụ. Catalog nằm trong
  `configs/experiments/swept_direction_cases.json`.
- [x] Đã chạy/kiểm tra bộ mở rộng 17 tình huống/612 update; báo cáo đủ từng ca,
  seed/phương án và lỗi. Đã kiểm tra 306 replay payload/liên kết, mở report trên
  Chrome và kiểm tra replay ca thất bại. Xem
  `artifacts/swept_volume_direction_evaluation/report.html`.
- [x] Chạy nhiều seed, so từng cặp cùng dữ liệu và giữ mọi ca thất bại.
- [x] Lưu protocol, CSV, source hashes, mức biến động giữa seed và báo cáo.
- [x] Tách đánh giá planner khỏi việc đàn có bay hoàn thành với controller cũ hay không.

Cần làm tiếp:

- [ ] Chọn trước tập map/seed mới, tách khỏi tập dùng để chọn tham số.
- [ ] Bổ sung hành lang hẹp, nhiều chướng ngại tĩnh và nhiều hướng cắt ngang.
- [ ] Kiểm tra ca vật cản rời tuyến, chặn hết lối, đi sát biên và gần các đầu tube.
- [ ] Giữ cùng ngân sách tính, điều kiện và bộ kiểm tra khi so các phương án.
- [ ] Báo kết quả theo từng loại tình huống, không chỉ trung bình gộp.
- [ ] Phân tích lỗi còn lại và kết luận đúng phạm vi. Các update trong cùng
  chuỗi có tương quan, không coi là các phép thử độc lập.

**Xong khi:** có bộ đánh giá mới được định nghĩa trước, kết quả tái lập được và
báo rõ điều kiện phương pháp thành công/thất bại.

**Sản phẩm cần xem:** cấu hình tập đánh giá mới, bảng kết quả và các replay lỗi.
Liên quan các mục cũ DT-11/12/19.

## B. Hướng 2 — Lập kế hoạch không gian–thời gian

**Trạng thái: chưa bắt đầu.** Đây là hướng nghiên cứu lớn thứ hai, không phải
một bước nhỏ đã hoàn thành trong hướng horizon. Các đầu việc dưới đây là khung
để nhớ; sẽ cụ thể hóa khi người dùng chọn bắt đầu hướng này.

- [ ] **B1. Chốt bài toán:** trạng thái, biến quyết định và giả thiết lịch tiến
  triển của UAV/đàn dọc tube. Tách tham số đường với thời gian thực.
- [ ] **B2. Biểu diễn vật cản theo thời gian:** kiểm tra chiếm dụng cùng vị trí
  cùng thời điểm, có xét bán kính, khoảng hở và bất định đã chọn.
- [ ] **B3. Xây planner:** tìm đồng thời đường/lịch đi qua; xác định có cho phép
  chờ hoặc thay đổi tốc độ không. Phải định nghĩa điều kiện biên và khả thi.
- [ ] **B4. So với hướng A:** cùng map, chuyển động vật cản, yêu cầu khoảng hở
  và budget; so tỷ lệ đạt, thời gian chờ, chất lượng đường và thời gian tính.

**Kết quả cần có trước khi gọi đã triển khai:** formulation và giả thiết rõ,
code chạy được, kiểm thử, bộ đối chứng và báo cáo giới hạn. Hiện chưa có các
kết quả này; không gọi capsule là một planner không gian–thời gian.

## C. Các phần mở rộng dùng chung — Để sau hoặc chờ chọn phạm vi

Các phần dưới đây không chen vào bốn bước A1–A4 nếu chưa được người dùng yêu cầu.

### C1. Cầu bao vật thể bất kỳ — Mới ghi nhận ý tưởng (DT-20)

- [ ] Chọn đầu vào: hình học đầy đủ, bounding box, mesh hay point cloud.
- [ ] Tính/kiểm tra cầu bao toàn vật thể; xét phần khuất và sai số khi có cảm biến.
- [ ] Đo độ bảo thủ với vật dài/mảnh và những ca mất đường do cầu quá lớn.
- [ ] Xét bán kính/orientation thay đổi và mô hình chuyển động ngoài vận tốc không đổi.

Cầu bao tổng quát hóa hình dạng, không tự giải quyết dự báo chuyển động. Chưa
triển khai backend tính cầu bao từ vật thể bất kỳ.

### C2. Sửa tube cục bộ và bàn giao cho đàn đang bay (DT-04/06/07/09/10)

- [ ] Chọn đoạn được sửa/phần giữ nguyên và thông tin trạng thái đàn cần dùng.
- [ ] Định nghĩa đoạn nối/miền chồng lấn, độ liên tục và điều kiện bàn giao.
- [ ] Kiểm tra đoạn chuyển tiếp, không chỉ candidate mới.
- [ ] Quản lý phiên bản, thời điểm kích hoạt và khả năng đàn còn nằm trong tube mới.
- [ ] Xác định giới hạn tube đơn nhánh; xử lý nhu cầu chia nhóm/nhập đàn nếu mở rộng.

Bản hiện tại lập lại toàn tuyến giữa hai đầu cố định, chưa giải quyết các mục này.
A3 xử lý thời gian và tính hợp lệ của candidate; không tự bao gồm toàn bộ bài
chuyển tiếp của đàn. Nếu phạm vi nghiên cứu tiếp tục chỉ là planner, công bố rõ
ranh giới này thay vì mặc định controller theo được.

### C3. Nhận thức, truyền thông và điều khiển thực (DT-13–17)

- [ ] Cảm biến, vùng nhìn, che khuất, tracking, mất track và sai số quan sát.
- [ ] Bản đồ local/tăng dần, xóa dấu vật cản cũ và xử lý vùng chưa biết.
- [ ] Chọn tính ở mặt đất, leader/scout, từng UAV hoặc kết hợp; đo tài nguyên.
- [ ] Hệ tọa độ, đồng bộ thời gian, trễ/mất gói, tuổi dữ liệu và mất leader.
- [ ] Outer cho tube chuyển động, tương tác hút/đẩy và phản ứng né cục bộ.
- [ ] Inner/plant: khả năng cơ động, trễ bám, giới hạn đầu vào và nhiễu.
- [ ] Kiểm chứng an toàn liên tục, khả thi đệ quy, latency toàn hệ thống và bay thật.

Lưu ý: SDF là biểu diễn khoảng cách, không phải bộ nhận diện. Drone chuyên trách
không thấy được phần bị khuất chỉ nhờ có máy tính mạnh. Có thể truyền tracks,
phần bản đồ thay đổi hoặc hệ số tube; không bắt buộc truyền cả lưới 3D. Kết nối
đủ cho telemetry chưa chứng minh đủ cho tránh vật cản realtime; mất liên lạc
cũng không mặc định hover là an toàn.

Nguồn đã tham khảo cho giai đoạn này:
[FIESTA](https://arxiv.org/abs/1903.02144),
[nvblox](https://nvidia-isaac-ros.github.io/v/release-3.1/concepts/scene_reconstruction/nvblox/technical_details.html),
[EGO-Swarm](https://arxiv.org/abs/2011.04183),
[Robust MADER](https://arxiv.org/abs/2303.06222).

## D. Tra cứu mã cũ và bằng chứng

Giữ bảng này để những ghi chú dùng mã DT-xx trước đây không mất ý nghĩa. Không
cần đọc bảng khi theo dõi tiến độ hằng ngày; dùng A0–A4, B1–B4 và C1–C3 ở trên.

| Mã cũ | Nội dung | Vị trí theo dõi mới |
|---|---|---|
| DT-01 | Đầu vào vật cản | A0; mở rộng ở C1 |
| DT-02 | Dự báo, horizon và đệm | A0, A2 |
| DT-03 | Chọn formulation | Hướng A đã chọn làm trước; hướng B chưa bắt đầu |
| DT-04 | Toàn tuyến/cục bộ | A0, C2 |
| DT-05 | Khi nào cập nhật | A0 giữ tube nếu đạt; A3, C2 cần mở rộng chính sách |
| DT-06 | Trạng thái đàn và chuyển tiếp | C2 |
| DT-07 | Kiểm chứng hình học | A0 đã có; A1 và C2 còn phần nâng cao |
| DT-08 | Deadline và thất bại | A3 |
| DT-09 | Giới hạn tube một nhánh | C2 |
| DT-10 | Tham số, phiên bản, kích hoạt | A3, B1, C2 |
| DT-11 | Tình huống thử | A0, A2, A4 |
| DT-12 | Đo lường và đối chứng | A0–A4 |
| DT-13 | Perception và bản đồ | C3 |
| DT-14 | Tính toán và truyền thông | C3 |
| DT-15 | Outer loop | C3 |
| DT-16 | Inner và plant | C3 |
| DT-17 | An toàn hệ thống và bay thật | C3 |
| DT-18 | Chất lượng đường | A1 |
| DT-19 | Đánh giá/realtime | A3, A4 |
| DT-20 | Cầu bao | C1 |

Bằng chứng chính: [PLANNER.md](PLANNER.md),
`artifacts/swept_volume_evaluation/`, `tests/test_swept_volume.py`.
Không sửa ngưỡng, clip state hay bỏ run thất bại để làm đẹp kết quả.

## Nhật ký ngắn

| Ngày | Quyết định hoặc kết quả |
|---|---|
| 2026-11-09 | Xác nhận bài Curve Virtual Tube là nền tảng; thí nghiệm repo là phần phụ |
| 2026-11-10 | Chọn nghiên cứu planner trước, giả sử có thông tin vật cản; outer/inner để sau |
| 2026-11-10 | Chọn thử hướng A trước; xây capsule và baseline lập tube |
| 2026-11-10 | Có cải tiến minimum snap và đánh giá 324 update; hình dạng tổng quát vẫn còn vấn đề |
| 2026-11-10 | Ghi nhận ý tưởng cầu bao; chưa triển khai |
| 2026-11-10 | Sắp lại tiến độ: hai hướng lớn A/B; bốn bước tiếp theo A1–A4 nằm trong horizon |
| 2026-11-10 | Mở rộng hướng chuyển động: 17 ca/612 update, 306 replay; 50 tests toàn repo đạt; A4 vẫn cần tập map/seed mới độc lập |
| 2026-11-10 | Thu hẹp trọng tâm về vùng quét/horizon: A1 là hạn chế planner nền để sau; làm A2 trước, không quy lỗi implementation thành lỗi bài báo |
| 2026-11-10 | A2 đã chạy 6 giá trị H, 540 update và 270 replay trong cùng điều kiện; chưa chọn H tối ưu hoặc quy tắc cho mọi môi trường |
| 2026-11-10 | Giới hạn A3 hiện tại ở báo cáo thông số, giả thiết dự báo và vấn đề độ trễ/tube lúc sử dụng; chưa yêu cầu triển khai có trễ hoặc bay thật |
