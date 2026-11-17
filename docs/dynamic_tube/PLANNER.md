# Planner tube động: né vùng vật cản quét qua

Ngày: 2026-11-10. Trạng thái: đã triển khai baseline planner-only cho sphere
vận tốc không đổi, chạy fixture và khảo sát 12 cấu hình. Chưa tích hợp điều khiển
đàn hoặc chuyển tiếp tube khi đang bay. Tài liệu này giữ formulation, giới hạn, implementation, lệnh chạy và kết quả;
các đề xuất cũ trùng với phần hiện thực đã được lược bỏ. Các cấu hình cụ thể là
lựa chọn thực nghiệm để tạo bản đầu, không phải tham số tác giả bài báo.

**Cập nhật cải tiến:** xem phần 12 về balanced durations, kiểm tra chất lượng,
ổn định tuyến và đánh giá đối chứng 324 update. Kết quả ở phần 9/11 là lịch sử
baseline trước cải tiến; report mặc định hiện dùng cấu hình cải tiến.

**Lộ trình tiếp theo:** xem [ISSUES.md](ISSUES.md), hướng A: A2 đánh giá/chọn
horizon → A3 xét thời gian tính → A4 kiểm tra map/seed mới. A1 (chất lượng của
planner nền) được ghi nhận nhưng tạm hoãn cải tiến tổng quát, không phải đóng
góp bắt buộc hoặc cổng trước A2. Hướng B (không gian–thời gian) vẫn chưa bắt đầu.
Các bước có checklist và
kết quả cần có trong ISSUES.md; không tạo thêm file kế hoạch riêng.

**Đánh giá theo hướng chuyển động:** phần 13 bổ sung 8 tình huống và báo cáo
đầy đủ từng trường hợp. Bộ 9 tình huống/324 update ở phần 12 giữ làm lịch sử.

## 1. Formulation đã chọn

Tại thời điểm lập kế hoạch tau, dự báo mọi vị trí vật cản có thể chiếm trong
horizon H, gom chúng thành một vùng cấm hình học. Tìm tube tránh vùng cấm đó.

Với vật cản cầu bán kính R_j, tâm hiện tại c_j và vận tốc v_j không đổi:

$$
\mathcal S_j=\bigcup_{h\in[0,H]}B(c_j+v_jh,R_j+m_j).
$$

Đây là capsule quanh đoạn từ c_j tới c_j+v_jH, với m_j là đệm dự báo. Vật cản
đứng yên cho capsule suy biến thành cầu. Với nhiều vật cản, vùng cấm là hợp các
capsule và vật cản tĩnh. Các công thức truy vấn và chứng nhận nằm ở phần 5.

```text
Trạng thái vật cản hiện tại + horizon
                 ↓
           Dự báo chuyển động
                 ↓
      Vùng quét của từng vật cản
                 ↓
     Kết hợp với bản đồ vật cản tĩnh
                 ↓
     Kiểm tra tube / tìm đường thay thế
                 ↓
       Làm trơn và tính lại bán kính
                 ↓
       Kiểm chứng → nhận hoặc từ chối
```

Planner không chọn thời điểm luồn qua một vị trí tạm trống; cả vùng quét bị cấm
trong bài toán tại lần cập nhật này. Đổi lại, chưa cần giải lịch chuyển động của
từng UAV. Không có nghiệm ở bài toán bảo thủ này không chứng minh nhiệm vụ gốc
không thể thực hiện bằng cách chờ hoặc chọn thời điểm khác.

## 2. Hai ca đặc biệt phải giữ trong kết quả

### 2.1. Vùng quét chứa vị trí hiện tại

Một vật cản sẽ bay qua vị trí drone sau vài giây có thể khiến vùng quét chứa
điểm bắt đầu ngay ở snapshot này, dù tại thời điểm hiện tại chưa va chạm.
Planner hình học bảo thủ có thể không tìm được đường thoát từ điểm đó.

Phải báo rõ trường hợp start/anchor nằm trong vùng cấm dự báo. Không bỏ kiểm
tra ở đầu đường, khoét lỗ capsule hoặc giảm horizon âm thầm để tạo nghiệm đẹp.
Muốn xử lý đầy đủ ca này có thể cần xét thời gian hoặc một cơ chế khác sau này.

### 2.2. Hết horizon và trễ tính toán

Tube tránh vùng quét chỉ liên hệ với khoảng dự báo đã dùng. Nếu dùng tube sau
đó, cần snapshot mới hoặc một kiểm tra tiếp nối; không mặc định an toàn mãi mãi.

Nếu dữ liệu tại tau nhưng candidate chỉ có thể dùng sau khi tính xong, phải xét
tuổi dữ liệu và khoảng thời gian hiệu lực thực. Bản đầu có thể giả thiết cập
nhật tức thời trong simulator, nhưng phải ghi rõ và chưa gọi đó là realtime
end-to-end. Tốc độ solver vẫn cần đo độc lập với giả thiết này.

## 3. Horizon, kích thước và vận tốc ảnh hưởng ra sao?

| Biến thay đổi | Hệ quả dự kiến cần đo |
|---|---|
| Bán kính vật cản tăng | Vùng cấm dày hơn, tube có thể phải đi xa hơn hoặc hẹp lại |
| Tốc độ tăng với H cố định | Capsule dài hơn; có thể chặn nhiều đường hơn |
| H tăng | Dự báo xa hơn nhưng vùng cấm bảo thủ hơn |
| H giảm | Dễ tìm đường hơn nhưng khoảng thời gian được xét ngắn hơn |
| Số vật cản tăng | Nhiều vùng quét hơn, truy vấn và tìm đường có thể tốn hơn |

Đây là xu hướng hình học, không phải cam kết mọi case đều có runtime hoặc kết
quả đơn điệu. Cần xét cả hướng chuyển động so với tube, không chỉ độ lớn vận tốc.

Không có một H đúng cho mọi cấu hình. H nên được khảo sát cùng chu kỳ cập nhật,
tuổi dữ liệu và phạm vi tube cần dùng; chưa chốt một giá trị tùy tiện làm chuẩn.

## 4. Implementation hiện có và trách nhiệm từng module

| File | Chức năng |
|---|---|
| `drone_bench/environment/swept.py` | Snapshot hình học, kiểm tra đầu vào, dựng capsule và truy vấn khoảng cách |
| `drone_bench/algorithms/planners/swept_volume.py` | Kiểm tra tube cũ, gọi baseline lập lại toàn tuyến, chứng nhận candidate, đo thời gian |
| `drone_bench/cli/plan_swept_volume.py` | Sinh quan sát tổng hợp, chạy chuỗi snapshot hoặc khảo sát tham số, xuất JSON/HTML |
| `configs/experiments/swept_volume.json` | World, planner, vật cản, horizon, thời điểm cập nhật và deadline |
| `tests/test_swept_volume.py` | Kiểm thử hình học, containment, các ca thất bại và integration |

Không sửa outer/inner, plant hoặc pipeline planner cũ để chạy thử. Snapshot kế
thừa giao diện `Environment`; `segment_free` dùng lại chứng nhận Lipschitz hiện
có, còn `clearance` kết hợp world tĩnh với capsule. Snapshot deep-copy dữ liệu
đầu vào nên sửa danh sách quan sát sau đó không làm thay đổi hình học đang giải.

Bản đầu đã chọn **lập lại toàn tuyến với hai anchor cố định** làm baseline.
Nếu tube cũ vẫn được chứng nhận trong snapshot mới thì giữ nguyên. Chưa tối ưu
đoạn cục bộ, chưa chạy UAV và chưa bảo đảm bàn giao tube động cho đàn đang bay.

## 5. Công thức triển khai chi tiết

### 5.1. Chuẩn hóa trạng thái theo timestamp

Với quan sát tại thời điểm t_obs và thời điểm snapshot tau:

$$
\Delta t=\tau-t_{\mathrm{obs}},\qquad
a=c_{\mathrm{obs}}+v\Delta t,\qquad b=a+vH.
$$

Chỉ chấp nhận quan sát không ở tương lai và không quá `max_age`. Trong CLI thử
nghiệm, quan sát mới được sinh ở mỗi thời điểm cập nhật, nên max_age bằng 0.
API hình học vẫn hỗ trợ ngoại suy quan sát cũ trong giới hạn tuổi cho phép.

### 5.2. Khoảng cách đến capsule

Đặt u=b-a. Nếu đoạn không suy biến:

$$
\alpha=\operatorname{clip}_{[0,1]}
\left(\frac{(p-a)^Tu}{u^Tu}\right),\qquad q=a+\alpha u.
$$

Khi a=b, đặt q=a. Khoảng hở với capsule j là:

$$
d_j(p)=\lVert p-q\rVert-(R_j+m).
$$

Khoảng hở truy vấn cho planner:

$$
d(p)=\min\{d_{\mathrm{static}}(p),d_1(p),\ldots,d_M(p)\}.
$$

Mỗi thành phần là 1-Lipschitz; phép lấy min giữ tính chất này. Ngoài vùng cấm,
đây là khoảng hở Euclidean đến tập vật cản/world. Bên trong vùng hợp chồng lấn,
không khẳng định đây là signed distance Euclidean chính xác đến biên hợp.

### 5.3. Chứng nhận đoạn nối trong RRT*

Với các điểm lấy mẫu dọc đoạn, khoảng cách giữa hai mẫu liền nhau là ell. Điều
kiện kiểm tra được dùng là:

$$
d(p_k)\geq r+\frac{\ell}{2}\quad\text{với mọi mẫu }k.
$$

Vì bất kỳ điểm nào trên đoạn cũng cách một mẫu không quá ell/2, tính 1-Lipschitz
cho một chứng nhận bảo thủ với khoảng hở yêu cầu r. Đây không phải chỉ kiểm tra
các mẫu rồi bỏ qua khoảng giữa chúng.

### 5.4. Tạo và co bán kính

Implementation baseline dùng bán kính waypoint khởi tạo:

$$
\rho_i=\min\{r_{\max},d(p_i)-m_r\},
$$

với `radius_margin` là m_r. Đường tâm và đường bán kính được fit bằng minimum
snap hiện có. Sau đó bán kính được scale đồng đều theo khoảng hở hình học và
`obstacle_margin`. Công thức khởi tạo/co này là lựa chọn của code trong repo,
không phải khẳng định tác giả bài báo công bố chính xác công thức đó.

Trong fixture: prediction_margin=2 m đã inflate capsule; radius_margin=3 m dùng
để khởi tạo bán kính; obstacle_margin=2 m là khoảng hở tube cần chứng nhận với
snapshot đã inflate. Đây là các lớp đệm riêng, được công bố rõ, không phải ba
lần cộng bán kính drone. Chưa có chứng minh các đệm này bao mọi sai số thực.

### 5.5. Chứng nhận tube đa thức liên tục

Tái sử dụng bộ chứng nhận Bernstein đang có trong repo. Với một đoạn đa thức,
chọn tâm bao q từ các control points đường tâm; E là bán kính bao bảo thủ của
convex hull; r_upper là control point bán kính lớn nhất. Kiểm tra:

$$
d(q)-E-r_{\mathrm{upper}}\geq m_{\mathrm{tube}}.
$$

Nếu chưa đạt, chia đôi đoạn bằng de Casteljau và thử lại. Với fixture này,
`m_tube` là obstacle_margin. Việc bao cả quả cầu quanh mọi điểm đường tâm cũng
bao đĩa tiết diện; có thể bảo thủ hơn kiểm tra đĩa trực tiếp.

Kiểm tra control points bán kính còn cung cấp điều kiện đủ cho giới hạn bán
kính trên toàn đoạn. Candidate không đạt chứng nhận sẽ bị từ chối; điều này
không tự chứng minh đường cong thực chắc chắn va chạm hay bài toán gốc vô nghiệm.

## 6. Cấu hình fixture hình học gốc

Phần này mô tả `configs/experiments/swept_volume.json`. Lệnh không tham số hiện
mặc định dùng config replay ở phần 11 để thấy tube đổi trong lúc phát, thay vì
fixture gốc chỉ đổi ngay tại t=0.

- World trống 500 × 500 × 300 m, cùng kích thước map với benchmark nhưng **không
  phải full benchmark có vật cản tĩnh của bài thử trước**.
- Start `(60,250,150)`, goal `(440,250,150)`; cả hai cố định qua các snapshot.
- Bán kính tube tối đa 25 m, floor 8 m.
- Vật cản 1: tâm ban đầu `(240,310,150)`, bán kính 9 m, vận tốc `(0,-20,0)` m/s.
- Vật cản 2: tâm ban đầu `(350,430,150)`, bán kính 3 m, vận tốc `(0,-25,0)` m/s.
- Horizon 3 s; các thời điểm cập nhật 0, 2, 5, 10 s.
- RRT* 350 iterations, seed ban đầu 17; mỗi update dùng seed=17+index.
- Deadline đo thử 1 s; đây là lựa chọn khảo sát, chưa phải yêu cầu điều khiển.

Các lần update là snapshot của một chuỗi thử nghiệm hình học; khoảng cách giữa
chúng không được tuyên bố là lịch thực thi liên tục an toàn. Không mô phỏng một
đàn bay trong các khoảng giữa snapshot. Đặc biệt có khoảng cập nhật dài hơn H.

`corridor_half_width=120` được chọn phù hợp đường nối dài với điều kiện đạo hàm
đầu/cuối bằng 0 của minimum snap. Trial ban đầu 25 và 60 làm QP báo singular ở
fixture này. Không sửa backend hay bỏ điều kiện để che lỗi; cấu hình cuối và
mọi trial sau được ghi trong resolved config. Corridor rộng không tự bảo đảm
an toàn; chứng nhận hình học sau tối ưu vẫn bắt buộc.

## 7. Các bước thực hành

Từ thư mục gốc repo, chỉ cần Python 3 standard library:

```bash
python3 -m unittest discover -s tests -p 'test_swept_volume.py' -v
python3 -m drone_bench.cli.plan_swept_volume
```

Mở `artifacts/swept_volume/report.html` bằng trình duyệt. Report hiện là replay
3D offline có chạy/dừng, tua thời gian, tốc độ 0.25–4×, chọn mốc update, xoay/zoom
và góc nhìn XY/XZ. Vật cản chuyển động liên tục, tube đổi tại mốc update được
ghi nhận; tube cũ được xóa khỏi hình khi thay bằng tube mới. Xem thêm phần 11.

Đổi cấu hình/đầu ra:

```bash
python3 -m drone_bench.cli.plan_swept_volume \
  --config configs/experiments/swept_volume.json \
  --output /tmp/my-swept-trial
```

Sao chép config sang một file riêng nếu thử thông số mới. Không sửa config hay
artifact của paper baseline để chạy swept planner.

Chạy khảo sát 12 cấu hình:

```bash
python3 -m drone_bench.cli.plan_swept_volume \
  --sweep --output artifacts/swept_volume_sweep
```

Bộ khảo sát dùng một số biến thể một yếu tố: radius 3/18 m của vật cản đầu,
tốc độ 5/45 m/s theo cùng hướng, horizon 1/6 s, một/không vật cản, stationary,
và hai ca start bị vùng cấm chứa. Đây là chẩn đoán có ca thất bại chủ đích;
`--sweep` trả mã 0 khi xuất xong, không có nghĩa mọi case thành công.

Lệnh thường trả 2 khi có update không được chấp nhận. Update quá deadline được
ghi riêng: mã 0 không chứng minh realtime, phải đọc deadline_misses và phạm vi đo.

Chạy regression toàn repo:

```bash
python3 -m unittest discover -s tests -v
```

## 8. Đọc artifacts và trạng thái kết quả

| File | Nội dung |
|---|---|
| `resolved_config.json` | Toàn bộ cấu hình dùng để tái lập |
| `source_hashes.json` | SHA-256 các module liên quan tại thời điểm chạy |
| `initial_tube.json` | Tube chỉ xét world tĩnh và báo cáo lập ban đầu, hoặc lỗi |
| `update_000.json`, ... | Quan sát hiện tại, capsule, khoảng hiệu lực, quyết định, timing, certificate và tube |
| `summary.json` | Số lần replan/retain/fail, thời gian và deadline misses |
| `report.html` | Replay 3D offline theo thời gian, timeline và trạng thái từng update |
| `sweep_summary.json` | Tổng hợp mọi cấu hình khảo sát, kể cả thất bại |

Các status chính:

- `retained`: tube cũ được chứng nhận lại với snapshot mới.
- `replanned`: candidate mới được chứng nhận.
- `start_blocked` / `goal_blocked`: anchor nằm trong vùng cấm dự báo/tĩnh.
- `start_too_narrow` / `goal_too_narrow`: không đủ chỗ cho radius floor + margin.
- `planning_failed`: không tìm được hoặc không chứng nhận được candidate trong
  lần thử này; xem error, không tự kết luận bài toán vô nghiệm.

`accepted=true` chỉ là chấp nhận hình học theo snapshot; không phải lệnh kích
hoạt ngoài đời. `live_activation_approved=false` và `realtime_demonstrated=false`
được giữ rõ. Deadline hiện được đo **sau khi giải xong**, chưa có hard timeout.

Nếu một update thất bại, output tube=null, CLI bỏ tube cũ khỏi ứng viên giữ lại.
Snapshot tiếp theo có thể tìm lại từ đầu. Không âm thầm dùng một tube vừa bị từ
chối hoặc mặc định u=0 là an toàn.

## 9. Kết quả thực chạy ngày 2026-11-10

Toàn bộ 46 tests đã qua, trong đó 8 tests mới kiểm tra: capsule/sphere suy biến,
vùng quét liên tục, timestamp và snapshot độc lập dữ liệu đầu vào, dữ liệu sai,
va chạm tube dù tâm không va chạm, va chạm giữa các mẫu, start bị chặn trong
tương lai, thất bại không tái sử dụng tube cũ, replan/retain và export.

Fixture mặc định: 4/4 snapshot đạt; 1 replan và 3 retain. Lần chạy lưu trong
`artifacts/swept_volume_fixture/summary.json` ghi số đo fixture gốc từng lần chạy.
Các lần kiểm tra
gần đây có mean update khoảng 0.14–0.16 s, max khoảng 0.56–0.64 s; 0/4 miss
deadline đo thử 1 s. Không suy ra p95/p99 đáng tin cậy từ chỉ
4 mẫu. Thời gian phụ thuộc máy và tải; không coi đây là benchmark phần cứng.

| Case khảo sát | Snapshot đạt | Snapshot thất bại | Replan | Retain |
|---|---:|---:|---:|---:|
| stationary | 4 | 0 | 0 | 4 |
| crossing_default | 4 | 0 | 1 | 3 |
| radius_3 | 4 | 0 | 2 | 2 |
| radius_18 | 4 | 0 | 2 | 2 |
| speed_5 | 4 | 0 | 2 | 2 |
| speed_45 | 4 | 0 | 1 | 3 |
| horizon_1 | 4 | 0 | 3 | 1 |
| horizon_6 | 4 | 0 | 1 | 3 |
| one_obstacle | 4 | 0 | 1 | 3 |
| no_obstacles | 4 | 0 | 0 | 4 |
| start_blocked_expected | 0 | 4 | 0 | 0 |
| future_start_conservative_failure | 2 | 2 | 1 | 1 |

Ca future_start cho thấy đúng giới hạn formulation: một vùng quét đi qua start
có thể khiến planner từ chối dù vật cản chưa chạm start tại thời điểm hiện tại.
Khi snapshot thay đổi và vùng quét không còn chứa anchor, planner có thể tìm lại.

Giới hạn timing: số đo update gồm kiểm tra/giải/chứng nhận nhưng chưa gồm dựng
snapshot, nhận thức, truyền thông hay export; thời gian dựng tube tĩnh ban đầu
được báo riêng. Chưa có kiểm chứng realtime end-to-end hoặc chuyển tiếp đàn.

## 10. Các việc chưa hoàn tất và thông tin cần khi mở rộng

- Chưa sửa cục bộ và chưa chọn anchor theo vị trí hiện tại của cả đàn (DT-04/06).
- Chưa đánh giá đầy đủ map tĩnh nhiều vật cản; fixture mặc định cố ý là world trống.
- Chưa hỗ trợ hộp xoay, quỹ đạo cong/gia tốc, bất định dự báo thay đổi theo thời gian.
- Chưa có hard timeout, tần suất cập nhật phù hợp tốc độ đàn, hoặc kích hoạt tube
  có tính đến trễ giải (DT-08/10).
- Chưa định vị chính xác đoạn xung đột/obstacle gây xung đột trong mọi ca: bản
  đầu báo lý do chứng nhận thất bại và lưu toàn snapshot để phân tích.
- Chưa thiết kế controller mới, handover cho UAV đang bay hoặc an toàn toàn hệ thống.

Không cần thêm dữ liệu từ người dùng để chạy fixture hiện tại. Trước khi đánh
giá cho nhiệm vụ cụ thể, cần chốt: kích thước/hình dạng vật cản thực muốn xét,
dải vận tốc, map, horizon/chu kỳ cập nhật, radius floor cho đàn, phần cứng và
ngân sách thời gian. Những thông số này chưa được thay bằng giả định nghiên cứu
chính thức chỉ vì bản demo đã chạy.

## 11. Replay theo thời gian: cách quan sát tube thay đổi

Report tĩnh ban đầu đã được thay bằng replay Canvas 3D. Hai file renderer là
`drone_bench/visualization/swept_replay.py` và `swept_replay.html`. HTML nhúng
toàn bộ config, tube ban đầu và các update; không cần server/CDN hoặc tải dữ liệu.

- Nút **Chạy/Dừng**, thanh tua và tốc độ phát điều khiển thời gian mô phỏng.
- Vật cản đỏ di chuyển theo vị trí/vận tốc trong config ở thời điểm đang xem.
- Tube xanh và các vòng tiết diện là tube của snapshot đang được chọn.
- Chỉ hiển thị một tube tại mỗi thời điểm; không chồng tube cũ và mới.
- Vùng quét vàng là capsule đã dùng ở lần giải đó, không trượt theo đồng hồ
  giữa các update; việc trượt sẽ hiển thị một dự báo planner chưa thực sự dùng.
- Các nút mốc thời gian giúp nhảy đến từng lần `replanned`, `retained` hoặc lỗi.
- Xem thời điểm vượt horizon sẽ có thông báo hết hiệu lực; hình học chỉ được
  giữ để quan sát. Khi update thất bại, không vẽ tube cũ như một tube được chấp nhận.
- Kéo để xoay, cuộn để zoom, chọn XY/XZ để xem rõ hướng tránh vật cản.

Không ép tube thay đổi mỗi frame: khi planner quyết định giữ tube, tube phải
giữ nguyên. Không nội suy giữa hai tube để tạo ảo giác đã có chuyển tiếp an toàn.
Các vòng tiết diện là hỗ trợ hiển thị bằng mẫu, không thay thế certificate 3D.

Để xem một demo có nhiều lần replanning, dùng config riêng (không thay số liệu
fixture gốc):

```bash
python3 -m drone_bench.cli.plan_swept_volume \
  --config configs/experiments/swept_volume_replay.json \
  --output artifacts/swept_volume_replay
```

Mở `artifacts/swept_volume_replay/report.html`. Config này dùng bán kính vật cản
đầu 3 m và các mốc 1, 2, 5, 7, 9, 10 s với horizon 3 s. Từ t=0 tới trước
t=1 s, hiển thị tube tĩnh ban đầu. Đúng mốc update, thay bằng kết quả planner
đã ghi. Vẫn chỉ là replay planner, không có drone bay.

### Quy tắc hiển thị đã đơn giản hóa

Theo yêu cầu người dùng, bỏ chế độ giải thích từng bước, bảng trước/sau và tube
cũ nét đứt. Replay giữ một tube duy nhất giữa các lần cập nhật. Tại mốc có tube
mới, canvas xóa hình cũ và vẽ tube mới; nếu retain thì tube giữ nguyên. Nếu update
thất bại thì không có tube được chấp nhận ở snapshot đó.

Demo bắt đầu ở t=0 với tube tĩnh thẳng; lần cập nhật động đầu tiên là t=1 s.
Đây là lịch cập nhật trong config demo, không phải trì hoãn hiển thị một kết quả
đã có ở t=0.

### Report mặc định và kiểm tra phát thực tế

Lệnh không tham số hiện dùng `swept_volume_replay.json` và ghi thẳng vào
`artifacts/swept_volume/report.html`, để tránh việc mở report mặc định cũ chỉ có
một lần replanning tại t=0 và không thấy đổi đường trong lúc phát. `--sweep`
không truyền config vẫn dùng fixture gốc để giữ bộ khảo sát nhất quán.

Đã kiểm tra trên Chrome bằng nút Chạy thực tế, chờ các callback animation của
trình duyệt, không gọi seek hoặc animate thủ công: t=0 vẽ tube thẳng với min Z
150 m; khoảng t=1.33 s vẽ tube của update đầu với min Z 66.45 m; khoảng t=2.35 s
vẽ tube của update tiếp theo với min Z 90.94 m. Đã chụp và xem ba ảnh canvas.
Kết quả kiểm tra và ảnh nằm trong `artifacts/swept_volume/verification/`.

Chạy lại fixture gốc khi cần đối chiếu:

```bash
python3 -m drone_bench.cli.plan_swept_volume \
  --config configs/experiments/swept_volume.json \
  --output artifacts/swept_volume_fixture
```

## 12. Cải thiện hình dạng và đánh giá đối chứng

### 12.1. Nguyên nhân đoạn vòng đã tái hiện

Trong demo ban đầu, waypoint RRT* tiến dần theo X nhưng đường sau minimum snap
có đoạn quay ngược. Thời lượng đoạn cũ tỷ lệ với chiều dài waypoint:

$$
T_k=\max(0.5,\lVert p_{k+1}-p_k\rVert/60).
$$

Snap cost theo tham số chuẩn hóa có hệ số tỷ lệ nghịch lũy thừa bảy của T_k.
Thời lượng chênh lệch làm thay đổi mạnh việc phân bổ độ trơn giữa các đoạn;
cùng corridor rộng có thể cho overshoot. Một snapshot giải xong không tự được
tối ưu tiếp trong replay: snapshot kế tiếp là một lần giải khác.

Đã thêm lựa chọn `duration_policy=balanced`:

$$
\widehat T_k=\max_j T_j\quad\text{cho mọi đoạn }k.
$$

Đây là thay đổi hình học tham số hóa cho planner phụ. Tổng thời lượng tham số
thay đổi, không phải tuyên bố UAV thực bay chậm lại hay cùng lịch arrival. Default
`distance` của paper planner được giữ để tái lập baseline.

Với đúng cùng waypoint của ca người dùng phát hiện:

| Đại lượng | Distance durations | Balanced durations |
|---|---:|---:|
| Chiều dài đường tâm | 519.406 m | 428.845 m |
| Quãng đi ngược theo hướng start→goal | Có | 0 m theo mẫu |
| Max curvature lấy mẫu (1/m) | Khoảng 0.267 | Khoảng 2.351 |
| Bán kính nhỏ nhất | Khoảng 20.461 m | Khoảng 25 m |
| Chứng nhận tube với world + capsule | Đạt | Đạt |

Test `test_same_waypoints_balancing_removes_observed_overshoot` tái hiện ca này
và kiểm tra việc cải thiện không đến từ thay waypoint. Không suy rộng rằng
balanced durations loại bỏ mọi vòng hoặc mọi điểm độ cong lớn.

**Tradeoff quan trọng:** max curvature lấy mẫu của chính ca này tăng sau đổi
durations, dù đoạn đi ngược và chiều dài giảm. Vì vậy chưa thể gọi đây là cải
thiện toàn diện về độ trơn/khả năng bám. Điểm tốc độ tham số gần 0 ở đầu/cuối
cần được xử lý cẩn thận khi phân tích curvature; chưa có bound hoặc radius ×
curvature criterion để chứng nhận tube không self-overlap. Không dùng hình
replay nhìn gọn hơn thay cho kiểm tra hình dạng này.

### 12.2. Metrics và kiểm tra chất lượng

Module `tube_quality.py` bổ sung các đại lượng tách khỏi certificate va chạm:

$$
L=\sum_i\lVert q_{i+1}-q_i\rVert,\qquad
\eta_L=\frac{L}{\sum_k\lVert p_{k+1}-p_k\rVert}.
$$

q_i là mẫu đường tâm; p_k là waypoint. Config cải tiến yêu cầu eta_L không quá
1.1. Đây là ngưỡng thực nghiệm cho độ dài tăng sau làm trơn, không phải chứng
minh injectivity hoặc giới hạn độ cong. Đường không đạt bị từ chối thay vì vẽ
như một kết quả hợp lệ.

Quãng đi ngược theo hướng start→goal được ghi như chẩn đoán:

$$
B=\sum_i\max(0,-(q_{i+1}-q_i)^Te),\qquad
e=\frac{p_{goal}-p_{start}}{\lVert p_{goal}-p_{start}\rVert}.
$$

B không được dùng làm hard constraint: đường vòng hợp lệ trong một map khác có
thể cần đi ngược hướng đó. B lớn cũng chưa đủ kết luận tự giao 3D.

Độ cong được đánh giá tại 64 midpoint tham số mỗi span:

$$
\kappa(u)=\frac{\lVert p'(u)\times p''(u)\rVert}{\lVert p'(u)\rVert^3}.
$$

Bỏ điểm có tốc độ tham số gần 0; đây là maximum lấy mẫu, chưa phải bound liên
tục, chưa áp dụng curvature hard limit. Cần đặc biệt thận trọng gần endpoint
có đạo hàm bằng 0.

Độ đổi tuyến giữa hai tube được đo bằng 65 cặp điểm tương ứng theo phần trăm
chiều dài đường tâm, báo mean/max khoảng cách. Đây là chỉ số hình học, không
phải sai số bám controller hoặc khoảng cách Hausdorff chính xác.

### 12.3. Ổn định giữa các update

Config replay cải tiến dùng seed cố định cho cả chuỗi để không đổi nguồn ngẫu
nhiên mỗi lần cập nhật. Khi tube cũ không đạt, có thể thử làm trơn lại waypoint
cũ bằng snapshot mới trước khi chạy RRT*. Waypoint cũ phải qua kiểm tra đoạn
nối; candidate vẫn phải đạt hình học và chất lượng. Thất bại được ghi vào
`attempts` rồi mới thử RRT*, không dùng lại tube chưa chứng nhận.

Trong bộ đối chứng bên dưới, thử reuse chưa có lần nào được nhận nên không
thể gán lợi ích thực nghiệm cho cơ chế reuse. Test riêng
`test_reuse_waypoints_resmooths_then_certifies` kiểm tra chức năng khi vật cản
mới làm thiếu khoảng hở tube nhưng waypoint vẫn hợp lệ.

Demo cải tiến hiện có 1 lần replan và 5 retain: đường tránh đầu tiên đã phù hợp
các snapshot tiếp theo. Không ép tube đổi tại t=2 chỉ để hoạt ảnh có chuyển động.

### 12.4. Giao thức đánh giá

Lệnh chạy:

```bash
python3 -m unittest discover -s tests -v
python3 -m drone_bench.cli.evaluate_swept_volume
python3 -m drone_bench.cli.plan_swept_volume
python3 -m drone_bench.cli.plan_swept_volume \
  --sweep --config configs/experiments/swept_volume_replay.json \
  --output artifacts/swept_volume_improved_sweep
```

Chín tình huống: crossing, large, fast, multiple, static_clutter, horizon 1 s,
horizon 6 s, start bị chặn, start nằm trong vùng quét tương lai. Mỗi tình huống
chạy 3 seed (7,17,29), 2 snapshot (t=1 và 2), 6 cấu hình: tổng 324 update.
Mỗi cấu hình có cùng budget 150 RRT* iterations, khác budget 350 của demo.

| Cấu hình | Mục đích |
|---|---|
| baseline | 41 | 41 | 0.273 | 6 |
| balanced_only | 42 | 42 | 0.270 | 2 |
| balanced_fixed_seed | 42 | 42 | 0.262 | 4 |
| improved | 42 | 42 | 0.261 | 4 |
| instantaneous | 48 | 4 | 0.269 | 2 |
| static_tube | Giữ tube từ world tĩnh, không né vật cản động |

Mọi candidate được đánh giá bằng **cùng full-horizon snapshot của từng case**,
kể cả instantaneous/static. Chứng nhận ở đây vẫn là hình học bảo thủ, không
khẳng định đã đánh giá plant. Các ca thất bại được giữ trong mẫu số.

Đây là exploratory suite trên map tổng hợp cùng họ với demo đã dùng khi phát
hiện lỗi, chưa có tập held-out độc lập và chưa đủ cho kết luận ưu thế thống kê.
Không coi 324 update là 324 phép thử độc lập; có tương quan theo seed/chuỗi.

### 12.5. Kết quả đã chạy

| Cấu hình | Candidate tạo được / 54 | Đạt common-horizon / 54 | p95 update (s) | Số update có B > 0.001 m |
|---|---:|---:|---:|---:|
| baseline | 41 | 41 | 0.273 | 6 |
| balanced_only | 42 | 42 | 0.270 | 2 |
| balanced_fixed_seed | 42 | 42 | 0.262 | 4 |
| improved | 42 | 42 | 0.261 | 4 |
| instantaneous | 48 | 4 | 0.269 | 2 |
| static_tube | 54 | 0 | 0 (không cập nhật online) | 0 |

Static có chi phí dựng ban đầu không nằm trong thời gian update bằng 0. Số B
đếm mọi candidate, còn các trung bình chất lượng trong summary chỉ tính kết
quả đạt common certificate; đọc tỷ lệ đạt trước khi so sánh trung bình.

Baseline có 12 start_blocked và 1 planning_failed. Improved có 12 start_blocked,
không che các ca mà formulation bảo thủ làm điểm bắt đầu nằm trong vùng cấm.
Không có miss deadline 1 s trong suite này, nhưng không suy ra realtime trên
mọi map/phần cứng hoặc end-to-end từ cảm biến tới plant.

Trên 41 update cùng đạt chứng nhận ở baseline và improved: chiều dài giảm trung
bình 26.777 m; chỉ số mean displacement giữa các tube giảm trung bình 14.614 m.
Chỉ balanced durations đã giảm chiều dài 27.024 m trên cùng 41 cặp; vì vậy không
gán toàn bộ lợi ích cho seed cố định/reuse. Các con số hình dạng không chứng
minh có bound robust hoặc an toàn controller.

Improved vẫn còn B khoảng 0.243–6.562 m ở 4 update của một số seed/case. Không
coi DT-18 hoàn thành tuyệt đối; chưa loại hết overshoot bằng một ràng buộc hình
học tổng quát và chưa có curvature/injectivity certificate.

48 tests toàn repo đã qua, gồm 2 test hồi quy mới cho overshoot và reuse.
Khảo sát improved 12 cấu hình/72 snapshot cũng đã xuất riêng, gồm ca thất bại.

### 12.6. Artifacts và tái lập

`artifacts/swept_volume_evaluation/` chứa:

- `protocol.json`: cấu hình từng case, seed, mode, platform và phạm vi đo.
- `source_hashes.json`: hash source của planner, geometry, numerical helpers.
- `details.json`: kết quả từng update, lỗi, certificate và diagnostics.
- `metrics.csv`: toàn bộ update, không lọc bỏ thất bại.
- `summary.json` và `report.html`: tổng hợp theo mode.
- `paired_comparison.json`: so sánh trên đúng cặp cùng đạt chứng nhận.
- `seed_variability.json`: min/max/std chiều dài qua seed ở cùng snapshot đầu;
  số seed đạt được ghi để tránh hiểu std có điều kiện là robustness đầy đủ.

Report replay mặc định là `artifacts/swept_volume/report.html`; cấu hình nằm
ở `configs/experiments/swept_volume_replay.json`. Fixture baseline vẫn dùng
`swept_volume.json`, không tự bật balanced policy cho baseline bài báo.

Công việc tiếp theo chưa giải quyết: curvature/local shape constraints phù hợp,
đánh giá held-out maps/seeds, hard timeout và chuyển tiếp cho đàn đang bay. Theo
dõi tại [ISSUES.md](ISSUES.md), đặc biệt DT-18.

## 13. Đánh giá vật cản đi theo nhiều hướng

### 13.1. Mục đích và cách chọn tình huống

Theo yêu cầu người dùng, thêm các hướng chuyển động để không chỉ đánh giá vật
cản đi cắt ngang từ một phía. Dữ liệu được đặt riêng trong
`configs/experiments/swept_direction_cases.json`, có tên, mô tả, tâm ban đầu,
bán kính và vector vận tốc của từng vật cản.

| Tình huống bổ sung | Chuyển động |
|---|---|
| direction_y_negative | Cắt ngang từ +Y về -Y |
| direction_y_positive | Cắt ngang theo hướng ngược lại |
| direction_x_positive | Đi dọc cùng hướng start → goal |
| direction_x_negative | Đi dọc ngược hướng start → goal |
| direction_z_negative | Từ trên đi xuống |
| direction_z_positive | Từ dưới đi lên |
| direction_diagonal | Đi chéo theo cả X, Y, Z |
| direction_converging | Ba vật cản cùng tới giữa tuyến từ ba hướng khác nhau |

Sáu ca theo trục dùng cùng tốc độ 20 m/s và bán kính 5 m; mỗi quả cầu đi qua
điểm `(250,250,150)` tại t=3 s. Tâm ban đầu được tính lùi từ điểm giao và vector
vận tốc. Nhờ đó so các hướng theo trục không đồng thời đổi kích thước và tốc độ.
Ca chéo có tốc độ khoảng 21.35 m/s, không phải một đối chứng cùng tốc độ tuyệt đối.
Ca hội tụ cố ý có nhiều bán kính và vận tốc; đánh giá riêng với các ca một vật.

Giữ 9 tình huống cũ và thêm 8 tình huống trên: 17 tình huống × 3 seed × 2 mốc
thời gian × 6 phương án = **612 lượt đánh giá**. Đây vẫn là map tổng hợp cùng
họ, chưa thay thế tập held-out độc lập hoặc thử nghiệm UAV thực.

### 13.2. Lệnh chạy và báo cáo

```bash
python3 -m drone_bench.cli.evaluate_swept_volume
```

Đầu ra mặc định mới: `artifacts/swept_volume_direction_evaluation/report.html`.
Kết quả bộ 9 ca trước đó trong `artifacts/swept_volume_evaluation/` vẫn được giữ.

Đổi catalog hướng và thư mục đầu ra:

```bash
python3 -m drone_bench.cli.evaluate_swept_volume \
  --directions configs/experiments/swept_direction_cases.json \
  --output artifacts/swept_volume_direction_evaluation
```

Mọi trường hợp đều phải xuất trong report, không chỉ những ca thành công:

- Mục lục đủ 17 tình huống, mỗi mục có tên, horizon, thông tin vật cản và vector vận tốc.
- Bảng tổng hợp theo phương án cho từng tình huống.
- Bảng chi tiết tất cả seed/phương án/mốc thời gian, trạng thái và nguyên nhân lỗi.
- Replay riêng cho từng seed/phương án: 17 × 3 × 6 = **306 replay**.
- Replay ghi riêng kết quả kiểm tra horizon chung; candidate của instantaneous
  hoặc static có thể được vẽ để xem nhưng không được hiểu là đạt safety.
- File `case_summary.json` cho tổng hợp từng tình huống; CSV/protocol/paired
  comparison/seed variability tiếp tục được giữ để phân tích và tái lập.

Replay được xuất từ chính dữ liệu đã đánh giá, không chạy lại một seed khác để
tạo hình minh họa. Nó giữ đúng hai mốc t=1 và t=2 s của bộ đánh giá, chưa đại
diện một lịch thực thi liên tục an toàn sau khi hết horizon.

### 13.3. Cách đọc kết quả

Xem tỷ lệ đạt horizon chung trước. Sau đó so chiều dài, độ cong, thời gian và
độ thay đổi tuyến trong cùng case/seed. Không kết luận một hướng tốt hơn chỉ vì
đường ngắn hơn trên các ca đã lọc thành công.

Trường hợp bị từ chối, singular QP, không có đường hoặc không đạt chứng nhận
vẫn phải được giữ đầy đủ. Chứng nhận không đạt chưa tự chứng minh vật thể thật
va chạm vì phép kiểm tra và vùng quét đều có tính bảo thủ.

### 13.4. Kết quả bộ mở rộng đã chạy

- Đã có đủ 612 dòng đánh giá và 306 replay; kiểm tra các payload, liên kết và
  source hashes. Đã mở report trên Chrome, đếm đủ 17 mục và kiểm tra riêng
  replay start_blocked không vẽ tube như thể được chấp nhận.
- Trong 8 ca hướng mới, improved đạt 48/48 lượt chứng nhận; baseline đạt 46/48.
  Hai lượt baseline không đạt thuộc ca cắt ngang -Y và ca chéo, đều được giữ
  trong bảng chi tiết với trạng thái planning_failed.
- Tính cả 17 ca: improved đạt 90/102 lượt; baseline 87/102; instantaneous 17/102;
  tube tĩnh 0/102. Mẫu số riêng của mỗi mode là 102, tổng 6 mode là 612.
- Không có deadline miss với deadline 1 s trong bộ này; thời gian vẫn chỉ tính
  phạm vi update đã nêu, không phải latency toàn hệ thống.
- 50 tests toàn repo đạt, gồm kiểm tra các hướng và kiểm tra report giữ đủ
  trường hợp/seed/mode/replay/lỗi.

Kết quả không chứng minh hết vấn đề độ cong/tube tự chồng lấn, không thay thế
held-out evaluation và chưa có controller hoặc đàn UAV thực.

## 14. A2 — Thay đổi horizon trong cùng điều kiện

### 14.1. Câu hỏi đang kiểm tra

Với cùng vị trí/kích thước/vận tốc vật cản, cùng map, seed, planner và lịch cập
nhật, chỉ thay horizon H thì khả năng tạo tube và đường tạo ra thay đổi thế nào?

Không tiếp tục sửa RRT*/minimum snap trong thí nghiệm A2. Dùng cố định cấu hình
planner đang có; các vấn đề chất lượng của planner nền ở A1 vẫn được công bố.

Về hình học, với cùng quan sát và mô hình vận tốc không đổi:

$$
\mathcal S_j(H)=\bigcup_{h\in[0,H]}B(c_j+v_jh,R_j+m),\qquad
H_1\leq H_2\Rightarrow\mathcal S_j(H_1)\subseteq\mathcal S_j(H_2).
$$

Do đó H tăng không làm vùng cấm nhỏ đi. Tuy nhiên, tỷ lệ solver tìm được nghiệm
hoặc chiều dài đường kết quả không nhất thiết biến thiên đơn điệu: thuật toán
lấy mẫu, bước làm trơn, các ngưỡng và việc giữ tube cũ cũng ảnh hưởng kết quả.

### 14.2. Cấu hình thí nghiệm

File `configs/experiments/swept_horizon_study.json` chứa:

- `horizons`: danh sách giá trị H được thử, mặc định 0.5, 1, 2, 3, 5, 6 s.
- `seeds`: 7, 17, 29; giữ đúng cùng seed cho các lựa chọn H trong một case.
- `reference_horizon`: 3 s, khoảng nhìn trước chuẩn dùng kiểm tra bổ sung.
- `last_update_interval`: 1 s, khoảng để kiểm tra sau snapshot cuối khi không
  có mốc kế tiếp trong dữ liệu. Đây là giả thiết được ghi rõ, không phải một
  chu kỳ điều khiển đã triển khai.
- `case_names`: null để chạy tất cả, hoặc danh sách tên case để chạy tập con.
- `base_config`: cấu hình planner được giữ cố định, có provenance trong outputs.

Dùng 15 tình huống từ bộ trước: bỏ hai case có tên horizon_1/horizon_6 để tránh
lặp vì H giờ là biến độc lập của toàn bộ phép thử. Giữ các hướng chuyển động,
vật cản lớn/nhanh/nhiều vật, map có vật cản tĩnh và các ca start bị chặn.

15 case × 3 seed × 6 giá trị H × 2 snapshot = 540 update và 270 replay.
Mỗi snapshot ở t=1,2 s; cùng budget 150 RRT* iterations. Hai update trong một
chuỗi có liên quan vì có thể giữ lại tube từ lần trước; không coi là mẫu độc lập.

### 14.3. Ba loại kết quả phải phân biệt

| Chỉ số | Ý nghĩa | Không được suy diễn |
|---|---|---|
| Đạt H đã chọn | Planner có tube đạt hình học trong vùng quét của H đó | Không bảo đảm sau khi hết H |
| Đạt tới update sau | Kiểm tra lại tube với vùng quét phủ khoảng giữa hai lần cập nhật | Chưa xét trễ tính/khả năng bám UAV |
| Đạt nhìn trước chuẩn 3 s | Kiểm tra bổ sung bằng cùng một khoảng nhìn trước, độc lập H đang chọn | H ngắn không đạt chỉ số này chưa phải bằng chứng nó sai hoặc có va chạm thật |

H ngắn hơn khoảng giữa các update được đánh dấu riêng để thấy khoảng thời gian
mà kế hoạch ban đầu chưa chứng nhận. Không âm thầm tăng H của planner để lấp
khoảng này; phép kiểm tra tới update sau là một đánh giá độc lập.

Các kiểm tra vẫn dùng swept-ball certificate bảo thủ. Candidate không chứng
nhận được chưa đồng nghĩa vật cản thật va chạm. Để chứng minh mức bảo thủ so với
chọn lịch đi qua, vẫn cần đối chứng thời gian phù hợp; hướng B chưa triển khai.

### 14.4. Thực hành và thay đổi H

```bash
python3 -m drone_bench.cli.evaluate_horizon
```

Mở `artifacts/swept_horizon_study/report.html`. Trên trang có ô chọn case, seed
và horizon; chọn H sẽ đổi replay và tô dòng tương ứng trong bảng so sánh. Các
kết quả đã được tính trước bằng Python, không giả lập chạy RRT* ở trình duyệt.

Thêm hoặc thay dải horizon:

```bash
python3 -m drone_bench.cli.evaluate_horizon \
  --horizons 0.5 1 1.5 2 3 4 6 \
  --output artifacts/my_horizon_study
```

Muốn chạy ít case/seed hơn, sao chép file study config, sửa `case_names`/`seeds`,
rồi dùng `--study <file-config>`. H phải dương, hữu hạn và không trùng. Không
ghi đè kết quả một phép thử khác mà không lưu được cấu hình của nó.

### 14.5. Dữ liệu và cách đánh giá

- `study_config.json`, `base_config.json`, `protocol.json`, `source_hashes.json`:
  lựa chọn H, điều kiện cố định, phạm vi đo và phiên bản code.
- `runs/<case>/seed_<seed>/H_<value>/`: mỗi lựa chọn có resolved config, tube
  ban đầu, snapshot, certificate, lỗi, thời gian và replay riêng.
- `evaluation_*.json` trong từng run: đánh giá thêm tới update kế tiếp và horizon
  chuẩn; lưu lý do/certificate, không thay kết quả planner gốc.
- `records.json`, `metrics.csv`: giữ tất cả update, cả những ca thất bại.
- `summary.json`: tỷ lệ đạt, thời gian mean/p95/p99/max và các chỉ số theo H.
- `paired_comparison.json`: so chiều dài trên cùng case/seed/thời điểm khi cả
  hai H đều có candidate; H tham chiếu lấy trong grid gần reference_horizon nhất.

Báo cáo có bảng so H trong cùng case/seed, biểu đồ chiều dài, bảng tổng hợp và
trạng thái mọi case. Các trung bình chiều dài chỉ tính trên candidate có được,
không gán chiều dài 0 khi thất bại. Cần xem tỷ lệ đạt trước rồi mới so chiều dài.

Giữ nguyên hạn chế: dữ liệu quan sát hoàn hảo, vận tốc không đổi, map tổng hợp,
chưa có held-out set, chưa mô phỏng đàn, chưa xét thời gian trễ trước kích hoạt.
Không dùng kết quả này để chọn một H tối ưu cho mọi môi trường.

### 14.6. Kết quả A2 đã chạy

Đã chạy đủ 540 update/270 replay, giữ mọi ca thất bại. Bảng dưới có 90 lượt
cho mỗi H, cùng các case/seed/mốc thời gian và một phiên bản planner:

| H (s) | Đạt trong H | Đạt tới update sau (1 s) | Đạt phép kiểm tra chuẩn 3 s | Chiều dài TB khi có tube (m) | p95 update (s) |
|---|---:|---:|---:|---:|---:|
| 0.5 | 81/90 | 38/90 | 20/90 | 394.91 | 0.460 |
| 1 | 81/90 | 81/90 | 27/90 | 399.80 | 0.457 |
| 2 | 78/90 | 78/90 | 52/90 | 409.26 | 0.459 |
| 3 | 78/90 | 78/90 | 78/90 | 412.29 | 0.451 |
| 5 | 78/90 | 78/90 | 78/90 | 418.33 | 0.446 |
| 6 | 78/90 | 78/90 | 78/90 | 415.69 | 0.439 |

Nhận xét trong đúng phạm vi bộ thử:

- H=0.5 ngắn hơn khoảng cập nhật 1 s. Số tube đạt H không phản ánh đầy đủ
  khoảng giữa hai lần cập nhật: 81 candidate đạt H nhưng chỉ 38 đạt kiểm tra 1 s.
- H=1 phủ khoảng cập nhật danh định trong mô hình lý tưởng. Chưa xét thời gian
  planner đang chạy, nên không được dùng kết quả này để kết luận H=1 đủ ngoài đời.
- H dài tăng phạm vi tránh, nhưng không tự tăng tỷ lệ tạo tube: ở H>=3, 12 lượt
  bị start_blocked. H=0.5/1 có 6 start_blocked và 3 planning_failed; H=2 có 9
  start_blocked và 3 planning_failed. Không quy mọi planning_failed thành vô nghiệm.
- Chiều dài trung bình có điều kiện không so trực tiếp như cùng tập thành công.
  Trên 78 cặp cùng có candidate với H=3, H=0.5 ngắn hơn khoảng 16.80 m;
  H=1 ngắn hơn 11.73 m; H=2 ngắn hơn 3.02 m; H=5 dài hơn 6.05 m;
  H=6 dài hơn 3.40 m. Không có quan hệ đơn điệu bắt buộc cho nghiệm solver.
- Không có miss deadline thử nghiệm 1 s. H dài không nhất thiết tính lâu hơn:
  số lần retain khác nhau; không suy ra độ phức tạp solver giảm theo H.

**Chưa chốt H tốt nhất.** Bộ này cho phép thấy tradeoff và chọn các ứng viên để
nghiên cứu tiếp. Muốn đề xuất quy tắc dùng thực tế cần thêm yêu cầu cập nhật/trễ
của A3 và đánh giá map/seed mới. Không xếp H ngắn là sai chỉ vì không đạt kiểm
tra xa hơn horizon mà nó được yêu cầu.

Đã kiểm tra 270 replay và config từng cặp chỉ khác H; source hashes khớp.
Trên Chrome, chọn H=0.5/H=6 tải đúng iframe, chọn seed thay dữ liệu tương ứng
và nhấn Chạy trong iframe đã đi qua update đầu. 53 tests toàn repo đạt.
Ảnh và kết quả kiểm tra browser lưu trong `artifacts/swept_horizon_study/verification/`.

## 15. Ranh giới A3 và cách mô tả kết quả hiện tại

Theo trao đổi với người dùng, giai đoạn này tập trung báo cáo thông số và giới
hạn của planner, chưa yêu cầu triển khai toàn hệ thống có trễ hoặc bay thực.

Cách mô tả phương pháp phù hợp:

> Từ vị trí, kích thước và vận tốc hiện tại của vật cản được cung cấp, hệ thống
> dự báo chuyển động trong một horizon theo mô hình vận tốc không đổi. Vùng vật
> cản có thể quét qua được xem là vùng cấm hình học tạm thời. Enhanced RRT* tìm
> đường tránh vùng cấm, minimum snap tạo tube, sau đó kiểm tra khoảng hở hình học.

Các thí nghiệm hiện tại dùng dữ liệu tổng hợp có trạng thái quan sát chính xác
và chuyển động đúng mô hình dự báo. Không gọi đó là nhận diện vật cản thực hoặc
khả năng biết chính xác tương lai. Khi vật cản đổi hướng/gia tốc hay thông tin
sai, vùng quét cần được thiết kế/kiểm chứng thêm.

Khi báo cáo thời gian, ghi cấu hình phần mềm/máy và phạm vi phép đo thật sự có
trong lần chạy. Các protocol hiện lưu platform, Python và thông tin processor
nếu có; chưa phải mô tả đầy đủ CPU/GPU/tài nguyên được cấp. Không điền bổ sung
một cấu hình phần cứng đoán từ máy khác hoặc suy ra khả năng chạy trên UAV.

Các hạn chế phải nêu rõ:

- Thời gian tính phụ thuộc phần cứng, implementation, tải máy, map và số vật cản.
- Replay hiện thay tube tại mốc dữ liệu, chưa cộng trễ tính vào lúc kích hoạt.
- Vật cản vẫn có thể di chuyển trong lúc planner tính; candidate hợp lệ với
  snapshot lúc bắt đầu không tự được bảo đảm hợp lệ tại lúc dùng trên UAV.
- Chưa có hard timeout, kiểm tra lại tube sau trễ hoặc xử lý thực thi khi tube
  cũ hết hiệu lực. Các trạng thái thất bại planner không tự là lệnh bay an toàn.
- Chưa kiểm chứng khả năng bám/chuyển tiếp tube của đàn, sai số cảm biến,
  truyền thông hoặc tính khả thi realtime toàn hệ thống.

Có thể đánh giá thuật toán trong mô phỏng mà chưa bay thực, nhưng kết luận chỉ
đúng phạm vi mô hình và bộ thử. Thử nghiệm trên phần cứng mục tiêu và thực tế là
bước sau khi muốn tuyên bố khả năng triển khai online, không phải bằng chứng
đã có của nghiên cứu này. Các việc còn mở được giữ tại A3 trong ISSUES.md.
