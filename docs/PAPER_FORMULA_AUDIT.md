# Đối chiếu công thức bài báo với mã thí nghiệm V-B

Ngày kiểm tra: 2026-11-10.

**Trạng thái:** nội dung dưới đây ghi nhận phiên bản trước sửa. Việc xử lý và
cấu hình mới được mô tả trong [PAPER_FORMULA_FIXES.md](PAPER_FORMULA_FIXES.md).

## Nguồn và phạm vi

Đọc PDF gốc tại:

```text
/home/tts/Downloads/1. Curve_Virtual_Tube_Planning_and_Distributed_Control_for_Swarm_UAVs_With_Complex_Constraints_and_Disturbances (1).pdf
```

Tên bài: *Curve Virtual Tube Planning and Distributed Control for Swarm UAVs
With Complex Constraints and Disturbances*, IEEE TIE 73(2), 3037–3048,
DOI 10.1109/TIE.2025.3603043.

Đã trích xuất lại PDF và xem ảnh trực tiếp các trang công thức 3042–3045 để
kiểm tra dấu, chuyển vị và định nghĩa. Đối tượng chính là `PaperOuter`,
`PaperInner`, potentials, plant, cấu hình V-B và vòng lặp mô phỏng. Đồng thời
đọc pipeline lập tube và minimum snap để xác định những sai khác hình học.
Không audit lại thuật toán DMPC: đó là thiết kế khác, không phải luật trong bài.

Không sửa controller, plant, tham số hay artifacts thí nghiệm cũ. Không thêm
giới hạn lệnh 9.5 m/s, safety filter hoặc một thành phần điều khiển mới.

## Kết luận

1. Chưa phát hiện lỗi chép dấu hoặc thiếu hạng tử trong các cập nhật inner
   (41), (42), (49), (50), (54)–(57), (60), (61) và luật thích nghi sau (66).
   Đặc biệt, không cộng trực tiếp `zeta_dot` vào H là phù hợp với cách bài
   gộp `-zeta_dot + Delta` vào bất định trong (43), (47).
2. Có sai khác thực sự ở hình học biên: code dùng khoảng hở hướng kính và
   hướng đẩy hướng kính tại một phép chiếu đường tâm; bài dùng điểm biên
   gần nhất trong định nghĩa trước (30). Hai cách không tương đương tổng quát.
3. Bộ hàm NN tự chọn trong code không tương thích với điều kiện đủ P > 0
   trong (77), khi đọc các norm theo chuẩn Euclidean/operator thông thường.
   Có thể chỉ ra mâu thuẫn bằng số, không cần chạy toàn bộ mô phỏng.
   Điều này không chứng minh hệ mất ổn định hoặc xác định duy nhất nguyên
   nhân vượt tốc; nó chặn việc viện dẫn trực tiếp điều kiện đủ này cho code.
4. Bài chỉ bão hòa lệnh vận tốc trong (2), (34). Phần inner kết luận boundedness
   trong (76)–(80), không đưa ra một bất đẳng thức riêng bảo đảm vận tốc thực
   luôn <= vm khi có sai số bám. Fig. 8 cũng không có đồ thị vận tốc thực.
5. Nhiều đại lượng của thí nghiệm là lựa chọn tái hiện, không phải giá trị
   tác giả công bố. Vì vậy không thể gọi hiện trạng là tái tạo chính xác
   thí nghiệm V-B, dù phần lớn công thức controller được chép đúng.

## 1. Bảng đối chiếu controller

Các đường dẫn dưới đây tính từ thư mục gốc repo.

| Công thức bài | Code | Kết quả đối chiếu |
|---|---|---|
| (1): p_dot=v; v_dot=f+g*u+Delta+d | `simulation/dynamics.py:5–12`, `controllers/paper_inner.py:47–49` | Khớp cấu trúc. f chứa drag và gravity; g=(1/m)I được tính bằng một scalar nhân từng trục. Giá trị vật lý và dạng Delta,d là tự chọn. |
| (2)–(3): bão hòa norm vận tốc lệnh | `controllers/paper_outer.py:7–9,74` | Khớp. Giới hạn áp sau tổng vector, không phải chỉ áp riêng guidance. |
| (4)–(5): hard saturation và tanh approximation | `controllers/paper_inner.py:37–39` | Khớp cách giữ cả hai: plant nhận hard saturation; hệ phụ dùng tanh. (59) giữ sai khác giữa chúng; không có cơ sở coi việc plant nhận hard saturation là lỗi. |
| (8): collision set | `controllers/paper_outer.py:67–68` | Kích hoạt khi distance < rs+ra; dấu bằng tại endpoint không đổi kết quả vì đạo hàm potential bằng 0 ở đó. |
| (9): cohesion set | `controllers/paper_outer.py:70–71` | Khớp ls <= distance <= lm. Việc không kéo cặp distance>lm là theo định nghĩa bài, không phải lỗi chép code. |
| (22): guidance curve | `controllers/paper_outer.py:41–54` | Đạo hàm có dạng tangent + offset*(dr/ds)/r, đúng cách dùng offset toàn cục của (22), nhưng thực hiện trên tube lấy mẫu thay vì đa thức liên tục. |
| (24): l'=l+lp, guidance saturation | `controllers/paper_outer.py:43–55` | Nhánh k1*lp>=vm là rút gọn đúng vì l>=0. Với cấu hình hiện tại guidance có norm 11 trước khi cộng các term khác. |
| (25)–(29): repulsive potential | `controllers/potentials.py:5–18` | Công thức và đạo hàm khớp; cặp UAV dùng safety=2rs, activation=rs+ra. |
| (30), (34)–(35): tube boundary | `controllers/paper_outer.py:25–33,58–60` | Potential scalar khớp; đối số khoảng cách và hướng vector chỉ tương đương trong hình học đặc biệt. Xem mục 2. |
| (31)–(33): cohesion potential | `controllers/potentials.py:21–28` | Khớp các nhánh mu, nu và đạo hàm thương. |
| (34): tổng outer và bão hòa cuối | `controllers/paper_outer.py:55–74` | Dấu guidance, pair repulsion và cohesion khớp sau khi khai triển dấu trừ ngoài saturation. |
| (41): filter | `controllers/paper_inner.py:16–19,25` | Khớp với tau=.15 I; zeta(0)=vc(0). Bài cho phép tau diagonal; code chọn ba phần tử bằng nhau. |
| (42), (49): e và z | `controllers/paper_inner.py:26–27` | Khớp e=v-zeta-Lambda và z=e-e_hat. |
| (47), (52): actor estimate | `controllers/paper_inner.py:28–31` | Khớp cấu trúc W_a^T B_a. Bản thân B, chuẩn hóa feature, tâm và bias là lựa chọn tự thêm khi hiện thực, không phải kiến trúc NN được V-B công bố. |
| (50): SPEM | `controllers/paper_inner.py:40–41` | Khớp P*z + f + g*u + actor + D_hat - Lambda_dot. |
| (54)–(55): cost estimate và actor update | `controllers/paper_inner.py:43–44` | Khớp. Code lưu W dạng feature-major; mỗi phần tử là B_j*(gamma_hat_k+e_k). |
| (56): critic update | `controllers/paper_inner.py:45` | Khớp B_c*(W_a^T B_a)^T nhân norm(e); dùng cùng basis cho actor và critic là một lựa chọn, không phải lỗi chuyển vị. |
| (57): LDO | `controllers/paper_inner.py:34,40,42` | Khớp xi_dot=z-C*(f+g*u+actor+D_hat-Lambda_dot), D_hat=xi+C*e. |
| (60): auxiliary system | `controllers/paper_inner.py:39` | Khớp -Ka*Lambda-g*(H-A(H)). Ka=2 I là tự chọn. |
| (61): H | `controllers/paper_inner.py:35–36` | Khớp cấu trúc; norm(g)^2=1/m^2 theo induced 2-norm của g=(1/m)I. Không nên tự thay bằng Frobenius norm khi PDF không ghi subscript F tại đây. |
| Sau (66): kappa_hat update | `controllers/paper_inner.py:32–33,46` | Khớp với Q=10 I và eta3=1; robust_smoothing=.5 ánh xạ sang hằng số smoothing trong tanh của (61), khác vector kappa_hat thích nghi. |

Các trường `controllers/...` trong bảng nằm trong
`drone_bench/algorithms/controllers/`; `simulation/...` nằm trong
`drone_bench/simulation/`.

## 2. Sai khác hình học biên đã xác nhận

Trang 3042 định nghĩa p_tilde_t = p - p_nearest, trong đó p_nearest là điểm
biên tube gần UAV nhất. (30) dùng norm của vector đó; (34)–(35) dùng cả hướng
vector để tạo lực đẩy.

Code không tìm điểm biên gần nhất. `PaperOuter.locate` tìm điểm gần nhất trên
đường tâm, rồi đặt:

```text
offset = p - centerline_projection
clearance = interpolated_radius - norm(offset)
boundary_velocity = V_t'(clearance) * offset / norm(offset)
```

Trong ống thẳng bán kính không đổi, hai cách khớp cả dấu và giá trị. Đã tính
ví dụ bán kính 40, p=(20,34,0), clearance=6:

```text
V_t'(6) = -34.92401587151726
boundary vector theo bài = (0,-34.92401587151726,0)
boundary vector của code = (0,-34.92401587151726,0)
lệnh sau saturation cả hai = (3.30462124,-10.49187678,0)
```

Vì vậy **không phát hiện lỗi dấu lực biên** trong ví dụ này.

Nhưng với đường tâm (0,0,0)->(100,0,0), bán kính r(x)=10+.5x,
p=(50,30,0):

```text
clearance code = 35 - 30 = 5 m
điểm gần nhất trên mặt bên = (48,34,0)
khoảng cách Euclidean = sqrt(2^2 + 4^2) = 4.472135955 m
```

Code đánh giá 5 m trong khi khoảng cách đến biên thật là 4.472 m; hướng tới
điểm biên gần nhất cũng có thành phần dọc trục mà cách đẩy hướng kính bỏ qua.
Đây là counterexample giải tích, không phải kết luận run hiện tại đã vi phạm
biên Euclidean. Muốn biết điều đó phải đo lại toàn bộ quỹ đạo với hình học đúng.

Ngoài ra `locate` chỉ tìm trong lân cận ±8 segment kể từ lần trước; không có
kiểm tra toàn cục bảo đảm kết quả vẫn là phép chiếu gần nhất ở tube tự gần nhau.
Chưa xác định điều này gây sai phép chiếu trong run hiện tại.

## 3. Điều kiện đủ P>0 trong (77) không thỏa với NN hiện tại

Đây là vấn đề giữa kiến trúc NN được tự chọn và điều kiện lý thuyết, không phải
lỗi dấu trong cập nhật trọng số.

Gọi q = k_(1,i)^2 là hằng số trong (69), (75), (77), **khác gain guidance
`outer.k1`**. Để mọi phần tử trong phép min của (77) dương, cần đồng thời:

```text
lambda_min(C) - q * B_bar_a^2 / 2 - 1 > 0
1 - k_(2,i)^2/4 - ||C||^2/(2*eta1*q) > 0
```

Với C=diag(1.9,1.9,2), eta1=1.1, theo induced 2-norm:

```text
lambda_min(C) = 1.9
||C|| = 2
```

Trong `paper_inner.py:29`, B bắt đầu bằng một bias bằng 1. Do đó norm(B)>=1
và bất kỳ upper bound B_bar_a hợp lệ nào cũng >=1. Điều kiện thứ nhất đòi:

```text
q < 1.8 / B_bar_a^2 <= 1.8
```

Điều kiện thứ hai, kể cả bỏ hạng không âm k_(2,i)^2/4 để lấy trường hợp dễ
nhất, vẫn đòi:

```text
q > 4/(2*1.1) = 1.818181818...
```

Hai khoảng không giao nhau. Dùng Frobenius norm của C sẽ làm điều kiện còn
chặt hơn. Như vậy không thể khắc phục chỉ bằng chọn lại các hằng số Young
k_(1,i), k_(2,i), khi giữ các gain và basis của code hiện tại.

V-B công bố các gain C, eta1 nhưng không cho kiến trúc, chuẩn hóa hay bound
norm của B. Kết quả này **không chứng minh cấu hình NN của tác giả cũng không
thỏa**, và **không chứng minh hệ code mất ổn định**: (77) là điều kiện đủ,
không phải điều kiện cần. Nó xác nhận không thể dùng nguyên điều kiện đủ này
để chứng nhận cấu hình NN mà repo tự chọn.

Không tự xóa bias hoặc đổi scale NN trong lượt audit: những thay đổi đó tác
động NN và cần được thiết kế, giải thích và kiểm chứng riêng.

## 4. Những lựa chọn tái hiện không do bài cung cấp

V-B ở trang 3046–3047 công bố N=27, vm=11, rs=5, ra=10, ls=100, lm=150,
P=diag(9.2,9.2,10.2), C=diag(1.9,1.9,2), K=diag(1.4,1.4,1.5),
Gamma_a=Gamma_c=.3 I, eta1=eta2=1.1. Code khớp các giá trị đó, cùng map bounds,
start, goal và bán kính tube tối đa.

Nhưng phần V-B không công bố đủ:

- Các gain k1–k4, lp, delta_n, delta_t của outer.
- tau filter, Ka, Q, eta3, smoothing trong robust term.
- NN: loại basis, số nút, tâm, độ rộng, chuẩn hóa và trọng số ban đầu.
- Mass, drag, input limit và dạng cụ thể Delta,d trong mô phỏng.
- Toàn bộ initial adaptive states, solver step và quy tắc rời miền ở đích.
- Tọa độ đầy đủ vật cản và các thiết lập planner mà repo đã tự chọn.

Các giá trị thay thế có trong `configs/experiments/paper_vb.json` và
`docs/PAPER_VB_IMPLEMENTATION.md`; không nên gộp chúng vào nhóm “lấy từ bài”.

Một điểm cần làm rõ thêm: `dynamics.py:10` cho Delta phụ thuộc cả p và v,
trong khi (47) viết Delta_bar là hàm của Z=[v,zeta,zeta_dot], không chứa p.
Với cùng Z, đổi p có thể làm Delta đổi, nên không được mặc định có một hàm
Delta_bar(Z) đơn trị đúng trên toàn không gian trạng thái của mô hình này.
Có thể phân bổ phần không biểu diễn được vào lumped residual, nhưng khi đó
phải xác lập bound và giả thiết của residual đó; audit này chưa có chứng nhận
như vậy. Đây là vấn đề mô hình/giả thiết, không phải thiếu một hạng p trong
luật NN mà bài yêu cầu.

## 5. Sai khác trong biểu diễn và lập tube

- (10)–(13) dùng đường đa thức và các tiết diện pháp tuyến. Planner của repo
  giải đa thức nhưng sau đó chuyển sang đường tâm và radius piecewise-linear
  để mô phỏng (`paper_tube.py:21–37`). Tangent/radius slope đổi tại đỉnh mẫu.
  Vì vậy không đồng nhất với dùng trực tiếp đạo hàm đa thức liên tục trong
  guidance (22). Nhận định cần xét ảnh hưởng đến bám, chưa đo riêng đóng góp
  của nó vào đỉnh vận tốc 12.523 m/s.
- `paper_tube.py:29–35` co bán kính sau QP để chứng nhận clearance với world.
  Đây là bước bổ sung có mục đích an toàn hình học, không phải công thức được
  cho trong Algorithm 1/(17)–(21).
- `minimum_snap.py` có mục tiêu snap + tracking phù hợp (20), ràng buộc giá
  trị/đạo hàm theo (17)–(18). Corridor được siết bằng Bernstein convex hull
  trên toàn segment, khác cách mô tả ràng buộc tại các điểm trong (19).
  Solver active set khác interior point được bài nhắc đến; đổi solver không
  tự nó là lỗi công thức nếu giải đúng cùng bài toán, nhưng các ràng buộc
  corridor siết thêm cần được công bố.
- Enhanced RRT* giữ cấu trúc MixCost và Visual trong (15)–(16), Algorithms
  2–3; khoảng lấy mẫu, clearance bổ sung và world cụ thể là lựa chọn tái hiện.

## 6. Liên hệ với vấn đề vượt vận tốc

Theo (41)–(42), với pi_filter=zeta-vc:

```text
v = vc + pi_filter + Lambda + e
```

Do đó norm(vc)<=vm chỉ cho:

```text
norm(v) <= vm + norm(pi_filter + Lambda + e)
```

Chưa có điều kiện ép phần sai số tổng hợp luôn hướng vào miền tốc độ an toàn.
Bound thực dụng cho Lyapunov trong (76)–(80) không đủ để thay thế điều kiện đó.

Trong (23), bài chuyển từ p_dot sang vc; trong khi (1) dùng p_dot=v. Đó là
bước phân tích outer tại mức vận tốc ảo. Nếu chuyển lại sang plant thực có sai
số bám, cần xét thêm phần do v-vc trong đạo hàm Lyapunov outer. Không được tự
suy từ kết quả outer lý tưởng và UUB của inner rằng vận tốc thực luôn <=vm.
Audit này không phải một phản biện đầy đủ toàn bộ chứng minh của bài.

Fig. 8(d)–(f) báo cáo khoảng cách cặp nhỏ nhất, khoảng cách biên nhỏ nhất và
khoảng cách cặp lớn nhất, không báo cáo extrema vận tốc thực. Vì vậy chưa có
dữ liệu trong bài để đối chiếu trực tiếp đỉnh tốc độ thực của repo với đỉnh
tốc độ thực của tác giả.

Những sai khác tìm thấy ở mục 2–5 là các vấn đề cần xử lý khi tái hiện; chưa
có thí nghiệm cô lập để quy đỉnh vượt tốc cho một sai khác cụ thể. Không có
cơ sở trung thực để kết luận đã tìm ra một dòng sai dấu trong inner mà sửa
xong chắc chắn hết vượt tốc.

## 7. Kiểm tra đã thực hiện

- Đọc lại PDF và xem trực tiếp các trang công thức, không chỉ dựa vào docs cũ.
- Tính ví dụ cylinder để kiểm tra dấu lực biên; kết quả code và (34) khớp.
- Tính counterexample hình nón để xác nhận radial clearance khác Euclidean.
- Tính các khoảng cần thiết của q từ (77); không giao nhau với bias NN hiện tại.
- Kiểm tra 30 trạng thái khác nhau bằng các đồng nhất thức độc lập:
  e_dot=v_dot-zeta_dot-Lambda_dot, z_dot=e_dot-e_hat_dot và
  D_hat_dot=xi_dot+C*e_dot. Dùng residual quan sát được
  R=Delta+d-zeta_dot-actor-D_hat, thu được z_dot=-P*z+R và
  D_hat_dot=z+C*R. Sai số lớn nhất 2.85e-14. Phép kiểm tra này xác nhận các
  dấu triệt tiêu trong SPEM/LDO, không giả vờ biết W* hoặc D tối ưu trong bài.
- `python3 -m unittest discover -s tests -v`: 29/29 test đạt. Các test này
  không chứng minh tái hiện chính xác hoặc thỏa mọi giả thiết lý thuyết.

Không chạy lại full mission với controller đã sửa trong lượt này, vì chưa
có thay đổi controller. Các số tốc độ 12.523 m/s và 12.445 m/s ở dt=.02/.01
đến từ artifacts đã có, không phải kết quả một bản sửa công thức mới.

## 8. Thứ tự xử lý phù hợp với mục tiêu tái hiện

1. Giữ vm=11 và các công thức controller; không thêm giới hạn lệnh thấp hơn
   để gọi đó là sửa bản tái hiện.
2. Làm rõ kiến trúc/scale NN sao cho có thể kiểm tra các giả thiết và điều
   kiện đủ của bài, hoặc ghi rõ không áp dụng được chứng nhận (77).
3. Nếu cần tái hiện đúng hình học bài: sử dụng trực tiếp đa thức đã giải và
   xác định biên gần nhất cho controller lẫn metrics. Phân biệt việc sửa
   hiện thực hình học này với thay đổi thiết kế luật điều khiển.
4. Kiểm tra từng thay đổi trên cùng input; đo cả tracking, filter, auxiliary,
   observer residual, tốc độ thực và độ nhạy bước thời gian. Không suy ra
   quan hệ nhân quả chỉ từ một full run mới đạt ngưỡng.
5. Nếu vẫn cần bảo đảm cứng cho vận tốc thực mà không suy ra được từ bài,
   phải nêu đó là yêu cầu mở rộng thiết kế, không phải lỗi đánh máy đã sửa.
