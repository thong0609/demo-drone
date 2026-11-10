# Sửa các sai khác hiện thực so với bài Curve Virtual Tube

Ngày: 2026-11-10. Cơ sở: `docs/PAPER_FORMULA_AUDIT.md` và PDF gốc tại
`/home/tts/Downloads`. Tài liệu audit mô tả phiên bản trước sửa; tài liệu này
mô tả cách xử lý các phát hiện, không tuyên bố tái tạo chính xác dữ liệu của tác giả.

## Các thay đổi

### Hình học biên theo (30), (34), (35)

`PaperOuter.locate` dùng điểm biên mặt bên gần nhất thay cho khoảng hở hướng
kính. Controller dùng đúng vector `p - p_nearest` trong hạng biên, và metrics
dùng cùng khoảng cách đó. Đĩa đầu/cuối là cửa vào/ra, không phải bức tường khiến
drone không thể đi qua đích.

- Tube thẳng tuyến tính theo đoạn: khoảng cách đến mặt nón cụt tính giải tích.
- Tube đa thức: với mỗi tham số, tìm chính xác điểm gần nhất trên vòng tròn
  pháp tuyến; sau đó tìm cực tiểu theo tham số liên tục trên tất cả các span.
  Tìm kiếm dùng nhiều khoảng khởi tạo (64 khoảng/span), golden-section với
  tolerance tham số 1e-9. Đây là cực tiểu số, không phải chứng nhận toàn cục
  bằng số học khoảng. Không còn cache chỉ tìm quanh ±8 đoạn.
- Với tube có self-overlap, các vòng tròn có thể không còn là biên ngoài của
  hợp các tiết diện. Chưa bổ sung chứng nhận injectivity cho tube; không suy
  rộng query mặt bên thành SDF của một tube tự giao bất kỳ.

### Dùng trực tiếp đa thức của (10)–(13), (22)

`PolynomialTube` giữ hệ số Bernstein degree 7, durations và tham số của các
mẫu. Guidance và phép chiếu đánh giá đa thức/đạo hàm liên tục, không dùng tangent
của polyline. Có xử lý giới hạn tiếp tuyến một phía khi vận tốc/đạo hàm bậc hai
ở endpoint bằng 0 theo (17). Khi cần chiều dài guidance trong (24), tích phân
đạo hàm (22) bằng Simpson; không thay bằng chiều dài dây cung.

Samples vẫn phục vụ hiển thị. Benchmark JSON xuất cả `tube.curve` và các samples;
load lại giữ nguyên đa thức và kiểm tra sự nhất quán với samples. JSON lịch sử
không có hệ số vẫn được đọc như tube lấy mẫu, không âm thầm gán một đường trơn
khác. Có tùy chọn CLI rõ ràng để khôi phục hệ số từ `planning.json` gốc; chỉ chấp
nhận nếu tái tạo được samples đã đóng băng.

Chứng nhận tránh vật cản kiểm tra đường cong liên tục bằng subdivision convex
hull của Bernstein và tính 1-Lipschitz của SDF, thay vì chỉ kiểm tra chord.
Hình vẽ vẫn là lưới hiển thị, không được dùng làm hình học điều khiển.

### Hàm NN phù hợp điều kiện đủ (77)

Bài không công bố kiến trúc hoặc chuẩn hóa B. Vì vậy đây là **lựa chọn hiện
thực có công bố**, không phải một giá trị mà tác giả đã cung cấp. Giữ nguyên
số feature và các gain P, C, K, Gamma_a, Gamma_c, eta1, eta2 được V-B công bố.

Với raw là bias + Gaussian RBF đang có, chọn:

```text
B(Z) = 0.5 * raw(Z) / sqrt(1 + norm(raw(Z))^2)
```

Hàm này trơn và norm(B)<0.5 trên toàn miền. Không thêm trạng thái điều khiển,
hạng lực hoặc giới hạn vận tốc mới. Các công thức cập nhật (54)–(61) giữ nguyên.

Chọn các hằng số Young `(k_(1,i),k_(2,i),k_(3,i),k_(4,i))=(2,1,1,1)` để
kiểm tra biểu thức (77), theo chuẩn Euclidean/operator 2-norm. Các hằng số này
phục vụ chứng minh, không phải các gain k1–k4 của outer.

Các phần tử trong phép min (77):

| Thành phần | Giá trị |
|---|---:|
| K | 1.8 |
| C | 0.8 |
| Actor | 0.65 |
| Critic | 1.65 |
| Filter | 12.333333 |
| Robust | 1.5 |
| SPEM | 18.4 |

Giá trị nhỏ nhất là 0.65>0. `PaperInner` từ chối cấu hình có khai báo witness
nhưng không thỏa positivity; CLI lưu `inner_certificate.json`.
Đây chỉ là kiểm tra positivity của **biểu thức được in trong (77)**, chưa
chứng minh toàn bộ giả thiết của định lý, các bound NN tối ưu, hoặc tốc độ thực
bất biến trong quả cầu 11 m/s.

Config cũ không khai báo chuẩn hóa vẫn chạy với raw basis để đọc lại lịch sử;
certificate ghi `verified=false`, không nhận là đã đạt điều kiện mới.

### Mô hình bất định phù hợp đầu vào Z của (47)

Phiên bản cũ dùng Delta_k=.15*sin(v_k/4+p_k/80), trong khi Z không chứa p.
Config mới chọn `uncertainty_model="velocity"`:

```text
Delta_k = .15*sin(v_k/4)
Delta_bar = Delta(v) - zeta_dot
```

Như vậy Delta_bar là hàm của Z=[v,zeta,zeta_dot] như (47). Biên độ bất định và
nhiễu giữ nguyên; dạng hàm cụ thể vốn là giả định của repo, không phải dạng
tác giả công bố. Đây là **thay đổi mô hình plant giả định**, cần công bố khi
so sánh, không thể quy toàn bộ thay đổi kết quả cho riêng controller.

Mô hình `position_velocity` vẫn được giữ cho config lịch sử; không đổi luật NN
bằng cách thêm p vào Z trái với định nghĩa bài.

### Khởi tạo và lưu lịch sử

Giữ nguyên vm=11, các ngưỡng vật lý và các gain được V-B công bố. Không clip
vận tốc sau tích phân, không thêm giới hạn lệnh 9.5 m/s.

Nếu dùng vị trí/vận tốc ban đầu cũ, bộ lọc và trạng thái thích nghi được khởi tạo
lại cho lệnh mới: zeta(0)=vc(0) theo (41), e_hat=e, xi=-C*e, trọng số/auxiliary/
robust gains ban đầu bằng 0. Không vừa thay lệnh ban đầu vừa giữ zeta cũ rồi
tuyên bố vẫn thỏa (41). `initialization.json` ghi rõ phần tái sử dụng.

- `paper_vb.json`: cấu hình đã sửa.
- `paper_vb_legacy.json`: bản config trước sửa, giữ để giải thích thí nghiệm cũ.
- Cấu hình so sánh DMPC trỏ tới legacy config; không tự áp kiến trúc NN mới lên
  predictor đã fit theo inner cũ. Tuy nhiên shared outer/metrics đã thay đổi;
  chạy lại không được coi là bit-identical với artifacts cũ. Gate cũ phải tạo lại.
- DMPC sampled/native reference từ chối benchmark đa thức để tránh âm thầm
  trộn geometry điều khiển liên tục với predictor chỉ hỗ trợ polyline.
- Thư mục mặc định mới là `artifacts/paper_vb_continuous`; kết quả kiểm tra sửa
  công thức nằm riêng ở `artifacts/paper_formula_fixes`.
- Mỗi lần chạy mới lưu source hashes và config. Không sửa artifacts cũ.

## Các lựa chọn không thể khôi phục từ bài

Vẫn chưa có dữ liệu tác giả về kiến trúc NN, các gain outer, tau, Ka, Q, eta3,
tham số vật lý, initial adaptive states hoặc tọa độ đầy đủ vật cản. Những giá
trị giả định đã khai báo trong config không tự trở thành tham số bài báo.

Co bán kính đồng đều sau QP được giữ như một lựa chọn lập tube bảo thủ và
được ghi rõ trong provenance. Nó tạo một radius polynomial mới vẫn được dùng
nhất quán trong cả controller lẫn chứng nhận; không phải công thức riêng mà
bài bắt buộc phải có. Không xóa kiểm tra tránh vật cản chỉ để làm pipeline
giống mô tả ngắn của Algorithm 1 hơn.

## Chạy lại

```bash
python3 -m unittest discover -s tests -v

python3 -m drone_bench.cli.run_paper_vb \
  --benchmark artifacts/paper_vb/benchmark.json \
  --restore-polynomial \
  --initial-state artifacts/paper_vb/initial_state.json \
  --output artifacts/paper_formula_fixes
```

Exit code 2 vẫn có nghĩa xuất kết quả nhưng còn tiêu chí không đạt; không đổi
ngưỡng hoặc gán success=true để che vượt tốc. Đọc `summary.json` và `report.html`.

## Kiểm tra

Các test bổ sung dùng nghiệm giải tích hình nón, đường cong so với công thức
vòng pháp tuyến lấy mẫu dày độc lập, tính liên tục tangent và endpoint, tìm
phép chiếu sau khi nhảy xa cache, serialize/restore hệ số, bound NN, từ chối
witness sai, sự độc lập của Delta với p, và một curve có bulge va chạm vật cản
giữa các điểm render. Toàn bộ suite tại thời điểm chạy có 46 test đạt, gồm
cả các test swept-volume đang có trong workspace.

## Kết quả chạy đầy đủ, dt=0.02

Nguồn: `artifacts/paper_formula_fixes/summary.json`.

| Chỉ số | Kết quả |
|---|---:|
| UAV hoàn thành | 27/27 |
| Thời gian hoàn thành | 62.825633 s |
| Tốc độ lệnh lớn nhất | 11.000000 m/s |
| Tốc độ thực lớn nhất | 12.429342 m/s |
| Khoảng cách cặp nhỏ nhất | 8.857723 m |
| Khoảng cách mặt bên nhỏ nhất | 5.367095 m |
| Đầu vào component lớn nhất | 30 N |
| Số bước vi phạm khoảng cách cặp | 53 |
| Số bước vi phạm tốc độ thực | 3068 |
| success | false |

Vi phạm cặp xảy ra trong khoảng 38.40–40.70 s (không phải mọi bước trong
khoảng đều vi phạm); nhỏ nhất tại 39.08 s. Tốc độ thực lớn nhất tại 40.86 s.
Các kết quả không chứng minh controller vật lý đạt tất cả ràng buộc sau sửa.
Việc sửa sự không tương thích giữa basis và (77) không đồng nghĩa đã có bound
tracking đủ nhỏ để tránh va chạm hay giữ tốc độ thực. Không tự đổi gain outer
hoặc ngưỡng để loại các vi phạm này.

`verification.json` xác nhận:

- Vị trí và vận tốc ban đầu của 27 UAV khớp dữ liệu cũ.
- Sai lệch zeta(0) so với vc(0) bằng 0.
- Source hashes khớp code lúc kiểm tra.
- 324 truy vấn mặt bên từ các frames cách nhau 5 s: đổi 64 lên 256 khoảng/span
  cho sai lệch khoảng cách lớn nhất 5.05e-13 m; điểm biên chênh tối đa 1.98e-6 m.
  Kiểm tra độ nhạy này không thay cho chứng nhận mọi cực tiểu toàn cục.
- Samples/radii hiển thị giữ nguyên chính xác so với benchmark cũ; phần thêm
  là hệ số đa thức đã lập ban đầu và chứng nhận geometry liên tục.

Lần chạy kiểm tra bước thời gian dt=0.01 được lưu riêng trong
`artifacts/paper_formula_fixes_dt001`, không trộn với lần chạy trên.

Kết quả đối chiếu đầy đủ:

| Chỉ số | dt=0.02 | dt=0.01 |
|---|---:|---:|
| UAV hoàn thành | 27/27 | 27/27 |
| Thời gian hoàn thành (s) | 62.825633 | 62.564216 |
| Tốc độ thực lớn nhất (m/s) | 12.429342 | 12.292488 |
| Khoảng cách cặp nhỏ nhất (m) | 8.857723 | 9.134826 |
| Khoảng cách mặt bên nhỏ nhất (m) | 5.367095 | 5.671861 |
| success | false | false |

Cả hai lần đều có vi phạm tốc độ thực và khoảng cách cặp. Giảm dt không loại
chúng; các con số vẫn nhạy bước thời gian, nên không tuyên bố đã kiểm tra hội
tụ liên tục. Dữ liệu máy đọc được ở
`artifacts/paper_formula_fixes/time_step_comparison.json`.

Kết luận triển khai: đã xử lý các sai khác hiện thực xác định được trong audit,
nhưng cấu hình giả định hiện tại **chưa đạt** thực nghiệm thỏa mọi ràng buộc.
Muốn tiếp tục tái hiện chặt chẽ, cần đối chiếu các tham số/kiến trúc chưa được
tác giả cung cấp và điều kiện áp dụng của chứng minh. Không coi việc tăng gain,
thêm giới hạn lệnh thấp hơn hoặc safety filter là sửa lỗi chép công thức.
