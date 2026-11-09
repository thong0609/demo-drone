# Context trao đổi: bài báo chính và hướng xét vật cản động

Cập nhật: 2026-11-09.

Tài liệu này lưu bối cảnh trao đổi giữa người dùng và trợ lý để tiếp tục công việc
ở các phiên sau. Nó phân biệt nội dung bài báo gốc, thí nghiệm phụ trong repo,
và ý tưởng mở rộng đang được thảo luận. Đây chưa phải đặc tả triển khai hay một
kết quả nghiên cứu đã được kiểm chứng.

## 1. Điều người dùng đã xác nhận

- Bài báo về **Curve Virtual Tube** trong `/home/tts/Downloads` mới là bài báo
  chính cần đọc hiểu và dùng làm nền tảng cho trao đổi.
- Các thí nghiệm hiện có trong repo là **thí nghiệm phụ của người dùng**. Không
  lấy DMPC v2 hoặc kết quả benchmark làm trọng tâm thay cho bài báo chính.
- Câu hỏi nghiên cứu hiện tại: outer loop của bài gốc gồm bốn thành phần; nếu
  xuất hiện vật cản động, ví dụ **một viên gạch bay vào giữa đàn drone**, thì hệ
  thống sẽ ứng xử thế nào và cần mở rộng ra sao?
- Người dùng yêu cầu ghi lại đầy đủ context vào một file. Chưa yêu cầu lập trình
  bộ tránh vật cản động, sửa controller, chạy mô phỏng hay chọn một phương pháp
  mở rộng cụ thể.

## 2. Bài báo chính và nguồn đã đọc

**Tên:** *Curve Virtual Tube Planning and Distributed Control for Swarm UAVs
With Complex Constraints and Disturbances*.

**Tác giả:** Mengshun Yuan, Mou Chen, Mihai Lungu, Tongle Zhou.

**Ấn phẩm:** IEEE Transactions on Industrial Electronics, vol. 73, no. 2,
February 2026, pp. 3037–3048.

**DOI:** 10.1109/TIE.2025.3603043.

**PDF gốc trên máy:**

```text
/home/tts/Downloads/1. Curve_Virtual_Tube_Planning_and_Distributed_Control_for_Swarm_UAVs_With_Complex_Constraints_and_Disturbances (1).pdf
```

Trợ lý đã đọc bản PDF 12 trang bằng trích xuất văn bản và xem trực tiếp trang
3044 chứa các phương trình inner loop (49)–(62). Bản trích xuất dùng trong phiên
ở `/tmp/main_virtual_paper.txt`; đây là file tạm, có thể không còn ở phiên sau.
Khi cần kiểm tra chính xác ký hiệu, dấu hoặc ma trận, phải đối chiếu PDF gốc vì
trích xuất văn bản có thể làm mất định dạng công thức. Việc đọc này chưa phải
một audit độc lập toàn bộ chứng minh của bài báo.

## 3. Bài toán và cấu trúc phương pháp của bài gốc

### 3.1. Mục tiêu

Đưa đàn UAV từ vùng xuất phát đến vùng đích trong môi trường 3D có vật cản thông
qua một curve virtual tube dùng chung. Đồng thời xét:

- Giới hạn tốc độ.
- Giới hạn đầu vào điều khiển/bão hòa actuator.
- Tránh va chạm giữa UAV.
- Giữ UAV trong tube với khoảng hở an toàn.
- Duy trì độ gắn kết đàn, tránh các UAV cách nhau quá xa.
- Bất định mô hình và nhiễu.

Bài báo giả thiết attitude đã ổn định và tập trung vào động lực học tịnh tiến:

```text
p_dot_i = v_i
v_dot_i = f_i + g_i u_c,i + Delta_i + d_i
```

Trong đó `u_c,i` là đầu vào tác động lên plant, khác với lệnh vận tốc ảo
`v_c,i` do outer loop sinh ra. Không đồng nhất hai đại lượng này.

### 3.2. Lập kế hoạch tube — Section III

Pipeline chính:

```text
Map → SDF → enhanced RRT* → benchmark path
    → minimum-snap curve generation → đường tâm p_b(t) và bán kính r_b(t)
```

- SDF cung cấp thông tin khoảng cách tới vật cản.
- Enhanced RRT* dùng mixed cost gồm độ dài đường và penalty khi gần vật cản,
  theo (15)–(16).
- Hàm `Visual` xét lại các ancestor để chọn parent có kết nối không va chạm và
  chi phí tốt hơn, theo Algorithms 2–3.
- Minimum snap tối ưu đường cong với chi phí snap và độ lệch so với đường
  benchmark; có ràng buộc đầu/cuối, nối các đoạn và corridor, theo (17)–(21).
- Tube là hợp các đĩa tròn vuông góc với tiếp tuyến đường tâm, theo (10)–(13).

Đây là cách lập một miền bay chung, giúp UAV linh hoạt trong tiết diện thay vì
bắt tất cả bám một đường tâm duy nhất.

### 3.3. Outer loop — Section IV-A

Người dùng đang tập trung vào **bốn thành phần** của luật outer:

| Thành phần | Vai trò |
|---|---|
| Guidance / vận tốc hướng tới | Đưa UAV tiến theo guidance curve tới miền đích |
| Lực đẩy giữa drone | Tránh va chạm giữa các UAV |
| Lực hút giữa drone / cohesion | Giữ các UAV không cách nhau quá xa |
| Lực đẩy khỏi thành ống | Giữ UAV bên trong tube, cách biên an toàn |

Trong trao đổi, từ “lực” được dùng để mô tả trực giác các trường hút/đẩy. Các
thành phần này kết hợp ở **mức lệnh vận tốc outer**, không phải trực tiếp là lực
vật lý tính bằng newton đưa vào plant.

Mỗi UAV có guidance curve riêng. Theo (22), có thể viết:

```text
p_guidance,i(t)
  = p_b(t) + r_b(t) * (p_i - p_b(t_q,i)) / r_b(t_q,i)
```

Offset trong công thức gốc được biểu diễn theo tọa độ toàn cục. Không tự thay
bằng một local frame được vận chuyển dọc tube khi nói đang tái hiện bài gốc.

Luật (34) kết hợp guidance, collision avoidance, tube boundary và cohesion,
sau đó bão hòa norm theo `v_m,i`. Các hệ số từ đạo hàm thế năng được định nghĩa
ở (35).

Các ngưỡng cần phân biệt:

- `r_s`: bán kính an toàn của một UAV; khoảng cách cặp mục tiêu tối thiểu là
  `2 r_s`.
- `r_a`: bán kính vùng tránh va chạm.
- `l_s`: khoảng cách bắt đầu tác động duy trì đàn.
- `l_m`: giới hạn khoảng cách đàn.
- Tập cohesion trong (9) xét các cặp có `l_s <= distance <= l_m`.

Phân tích outer thu được `V_dot_o <= 0`, theo (36)–(40), trong phân tích vận tốc
ảo của bài. Không tự suy ra từ đó một bảo đảm an toàn cứng cho plant thực có sai
số bám, trễ và lấy mẫu.

### 3.4. Inner loop — Section IV-B

Inner loop bám lệnh vận tốc outer dưới bất định, nhiễu và bão hòa đầu vào:

- Bộ lọc bậc nhất cho `v_c,i`, theo (41).
- Sai số bám có bù hệ phụ: `e_i = v_i - zeta_i - Lambda_i`, theo (42).
- NN và actor–critic dùng để ước lượng bất định, theo (45)–(56).
- SPEM cung cấp sai số dự báo `z_i = e_i - e_hat_i`, theo (49)–(51).
- Learning-based disturbance observer (LDO) ước lượng nhiễu tổng hợp, theo (57).
- Hệ phụ xử lý ảnh hưởng bão hòa đầu vào, theo (60).
- Luật điều khiển inner ở (61), cùng cập nhật thích nghi được trình bày sau (66).

Phân tích inner thu được:

```text
V_dot_e <= -P V_e + G
```

theo (76), với điều kiện chọn tham số để `P > 0`. Đây là bound thực dụng; không
diễn giải thành mọi sai số nhất thiết hội tụ chính xác về 0.

### 3.5. Thí nghiệm trong bài gốc — Section V

- **V-A:** so sánh 2D với 30 UAV giữa controller đề xuất và traditional vector
  field; dùng cùng inner loop để so outer.
- **V-B:** mô phỏng 3D với 27 UAV, map 500 × 500 × 300 m; start
  `(50,140,50)`, goal `(460,390,170)`; bán kính tube tối đa 45 m;
  `v_m = 11 m/s`, `r_s = 5 m`, `r_a = 10 m`, `l_s = 100 m`, `l_m = 150 m`.
- **V-C:** thí nghiệm bay trong nhà với motion capture, workstation điều khiển
  và truyền lệnh WiFi.

Trong kết luận, tác giả nêu tạo tube thời gian thực trong môi trường động và
chiến lược điều khiển phân tán nâng cao là hướng phát triển tương lai.

## 4. Vai trò của repo hiện tại: thí nghiệm phụ

Repo `/home/tts/prj/simulate-drone` có benchmark hình học, pipeline planner,
controller và simulator để thử nghiệm, đối chiếu và mở rộng. Đây là phần hỗ trợ
nghiên cứu của người dùng, không thay thế nội dung bài gốc.

### 4.1. Kiến trúc đã đọc từ tài liệu

- `environment/`: world, vật cản, clearance.
- `tubes/`: biểu diễn tube, phép chiếu, kiểm tra hình học.
- `algorithms/planners/`: reference, enhanced RRT*, minimum snap, paper tube.
- `algorithms/controllers/`: paper outer, paper inner, potentials và DMPC.
- `simulation/`: khởi tạo, dynamics, vòng lặp, interfaces và metrics dùng chung.
- `visualization/`: export, report và replay offline.
- `configs/`: scenario và cấu hình thí nghiệm.
- `artifacts/`: đầu ra sinh tự động, không sửa số liệu bằng tay.

Cần phân biệt benchmark reference dùng đường sinusoidal với pipeline paper tube
dùng enhanced RRT* và minimum snap; không gọi đường sinusoidal là kết quả planner
theo bài báo.

### 4.2. Các kết quả phụ được tài liệu ghi nhận

Các số dưới đây lấy từ docs trong phiên trao đổi; trợ lý **chưa chạy lại mô
phỏng hoặc tự kiểm chứng artifacts** trong phiên này.

| Chỉ số | Baseline vector field | DMPC v2 |
|---|---:|---:|
| Hoàn thành | 27/27 | 27/27 |
| Thời gian hoàn thành | Khoảng 60.624 s | Khoảng 72.525 s |
| Tốc độ thực lớn nhất | Khoảng 12.523 m/s | Khoảng 9.932 m/s |
| Khoảng cách cặp nhỏ nhất | Khoảng 12.000 m | Khoảng 10.347 m |
| Local radial clearance nhỏ nhất | Khoảng 8.796 m | Khoảng 13.876 m |

Baseline hoàn thành nhưng vượt ngưỡng tốc độ 11 m/s. DMPC v2 hoàn thành và không
có vi phạm được đo theo metrics hiện tại.

DMPC là thiết kế riêng cho benchmark, phối hợp Gauss–Seidel, không phải một bản
tái hiện ADMM/DCBF. Nội dung ADMM và DCBF trong thư mục presentation là phần lý
thuyết bổ sung; không có nghĩa chúng đã được triển khai trong simulator.

Các giới hạn đã nắm:

- DMPC chưa đạt real time: mean outer update khoảng 1 s so với period 0.2 s;
  tài liệu ghi 353/363 update trễ deadline.
- Plant mô phỏng chưa nhận lại độ trễ tính toán này như trễ thực thi.
- Baseline và DMPC khác reference speed và outer update period; chưa cô lập
  riêng lợi ích của cơ chế tối ưu.
- Clearance đang dùng là local radial clearance, không phải Euclidean signed
  distance chính xác đến toàn bộ bề mặt tube.
- Không có vi phạm được đo không tương đương chứng minh an toàn liên tục.
- Các kết quả v1 là lịch sử; DMPC v2 đã thay đổi formulation preferred buffer
  và bổ sung inter-knot collision cuts.

## 5. Câu hỏi hiện tại: viên gạch bay vào giữa đàn

Người dùng đặt tình huống một vật cản động từ bên ngoài bay vào giữa đàn drone
đang di chuyển trong tube. Chưa cho cụ thể kích thước, quỹ đạo, tốc độ viên gạch,
khả năng phát hiện hay giới hạn cơ động của drone.

### 5.1. Nhận định về outer gốc

Trong bốn thành phần đang xét, chưa có thành phần riêng biểu diễn tương tác với
một vật cản động như viên gạch.

Vật cản của bản đồ được xử lý từ bước lập kế hoạch để tạo tube trong vùng trống.
Khi vật cản mới xâm nhập tube, tính chất vùng trống theo bản đồ dùng lúc lập kế
hoạch có thể không còn đúng. Lực đẩy giữa UAV không tự nhận biết viên gạch, và
lực đẩy khỏi thành tube chỉ phản ứng với biên tube đã định nghĩa.

Nhiễu `d_i` trong dynamics và một vật thể có nguy cơ va chạm là hai đối tượng
khác nhau: có LDO bù nhiễu không tự cung cấp cơ chế phát hiện và né vật cản.

### 5.2. Ý tưởng mở rộng thứ nhất: thành phần tránh vật cản động

Trợ lý đề xuất có thể thêm một thành phần thứ năm vào tổng vector vận tốc:

```text
v_c,i = sat_v_m(
    v_guide,i
  + v_repulsion,i
  + v_cohesion,i
  + v_boundary,i
  + v_obstacle,i
)
```

Đây là biểu diễn trực giác theo các vector đóng góp, không phải chép nguyên ký
hiệu/dấu của (34). `v_obstacle,i` là ý tưởng đề xuất, **chưa có công thức thế
năng, gain, luật kích hoạt hay implementation được chọn**.

Cần xét khoảng cách giữa drone và bề mặt vật cản, có tính bán kính an toàn drone
và kích thước vật cản. Nếu dùng vật cản bao cầu, phải nêu rõ đó là một xấp xỉ hình
học bảo thủ phù hợp; không dùng khoảng cách tâm như thể vật cản là điểm.

### 5.3. Cần xét vận tốc tương đối, không chỉ khoảng cách

Hai vật cản ở cùng khoảng cách nhưng một đang đi xa và một đang lao tới tạo ra
nguy cơ khác nhau. Trợ lý đưa ra ví dụ dự báo đơn giản:

```text
r_io = p_i - p_o
v_io = v_i - v_o
```

Với giả thiết vận tốc tương đối không đổi trong horizon ngắn `T`, thời điểm gần
nhau nhất là:

```text
t_star = clip_[0,T]( -dot(r_io, v_io) / norm(v_io)^2 )
```

áp dụng khi `norm(v_io) > 0`. Vector tương đối dự báo tại đó là
`r_io + t_star * v_io`; norm của nó là khoảng cách tâm dự báo. Để đánh giá va
chạm còn phải xét kích thước và hình dạng hai đối tượng. Khi vận tốc tương đối
bằng hoặc gần 0, cần xử lý riêng để tránh chia cho 0 và nhạy số.

Đây là minh họa cách phát hiện nguy cơ sớm, chưa phải predictor đã kiểm chứng.
Viên gạch có thể có gia tốc, rơi theo trọng lực, xoay hoặc đổi hướng; mô hình vận
tốc không đổi chỉ là xấp xỉ trong khoảng ngắn và chưa được chọn cho thí nghiệm.

### 5.4. Các xung đột giữa thành phần outer

Khi vật cản lao vào giữa đàn, những mục tiêu hiện có có thể xung đột:

- Tránh viên gạch có thể đẩy UAV về phía UAV khác.
- Cohesion có thể kéo UAV trở lại vùng cần tránh.
- Giữ trong tube hạn chế hướng né, nhất là khi vật cản chiếm phần lớn tiết diện.
- Guidance vẫn thúc đẩy đi tới trong khi tình huống cần giảm tốc hoặc nhường đường.
- Inner và plant có thời gian đáp ứng hữu hạn; thay đổi lệnh vận tốc không làm
  UAV đổi vận tốc tức thời.

Vì vậy, tổng các vector hút/đẩy sau bão hòa chưa đủ để kết luận tất cả ràng buộc
an toàn đồng thời được thỏa mãn. Đây là vấn đề cần thiết kế và kiểm chứng thêm.

### 5.5. Hành vi mong muốn được đề xuất

Trợ lý gợi ý đàn có thể tạm tách để nhường đường cho vật cản, rồi gom lại sau khi
nguy cơ qua đi. Trong lúc né, có thể cần giảm ưu tiên cohesion và tiến về đích.

Đây là đề xuất điều chỉnh hành vi, chưa phải quyết định của người dùng. Đặc biệt,
**giảm lực cohesion không tự có nghĩa được phép bỏ ngưỡng khoảng cách đàn**. Nếu
muốn cho phép vi phạm tạm thời một yêu cầu, phải định nghĩa rõ trong bài toán và
metrics, không âm thầm đổi tiêu chí.

Nếu vật cản chặn hết tiết diện khả dụng, có thể phải giảm tốc hoặc chờ nếu còn
đủ thời gian và không gian. Nếu phát hiện quá muộn, tốc độ vật cản quá lớn hoặc
khả năng cơ động không đủ, có thể không tồn tại phương án tránh va chạm khả thi.
Không nên hứa rằng thêm một lực đẩy luôn giải quyết được tình huống.

### 5.6. Các hướng thuật toán đã nhắc đến

| Hướng | Ý tưởng | Trạng thái |
|---|---|---|
| Enhanced vector field mở rộng | Thêm tránh vật cản động có dự báo và xem xét điều chỉnh cohesion | Đề xuất gần outer gốc nhất |
| DMPC có vật cản động | Đưa chuyển động vật cản vào dự báo và ràng buộc tránh va chạm | Mới nêu khả năng mở rộng |
| Lớp lọc an toàn CBF | Điều chỉnh lệnh danh định để xét các ràng buộc an toàn | Mới nêu khả năng mở rộng |

CBF cần thiết kế phù hợp với cấp đầu vào, động lực học, giới hạn điều khiển,
bất định và lấy mẫu. Không mặc định một CBF ở mức vận tốc bảo đảm an toàn cho
plant thực do inner điều khiển. Chưa chọn CBF/HOCBF/DCBF, chưa chốt formulation,
và chưa có bảo đảm lý thuyết cho tình huống đang bàn.

Đề xuất gần nhất của trợ lý là: **giữ tube cố định, thêm tránh vật cản động có
dự báo, nghiên cứu cách điều chỉnh cohesion khi đàn phải tách để né**. Người
dùng chưa xác nhận chọn hướng này làm phương pháp triển khai.

## 6. Những điểm còn mở trước khi thiết kế thí nghiệm

Các nội dung sau chưa được quyết định trong trao đổi:

1. Mục tiêu là hiểu ý tưởng, thiết kế phương pháp nghiên cứu, hay hiện thực mô phỏng.
2. Giữ tube cố định hay cho tube thay đổi/replan khi vật cản xâm nhập.
3. Mô hình viên gạch: hình hộp hay bao cầu, kích thước, vận tốc, gia tốc, hướng bay,
   thời điểm xuất hiện và chuyển động quay.
4. Drone biết vật cản qua cảm biến hay thông tin mô phỏng; phạm vi phát hiện,
   chu kỳ cập nhật, nhiễu đo, độ trễ và khả năng chia sẻ thông tin.
5. Dự báo vật cản theo mô hình nào và với horizon bao lâu.
6. Giữ cứng những ràng buộc nào; xử lý ưu tiên cohesion/progress khi né ra sao.
7. Cách phối hợp để nhiều drone né mà không gây va chạm dây chuyền.
8. Dùng thêm trường đẩy, tối ưu dự báo, safety filter hay một tổ hợp cụ thể.
9. Cách đánh giá: khoảng cách bề mặt với vật cản, khoảng cách cặp, biên tube,
   tốc độ thực, input, completion, độ tách đàn, thời gian tính toán và các run lỗi.

Danh sách này lưu các phần chưa xác định, không có nghĩa cần hỏi hết trước khi
tiếp tục thảo luận khái niệm.

## 7. Hướng dẫn dùng context cho phiên tiếp theo

- Bắt đầu từ **bài báo chính và câu hỏi về outer bốn thành phần**.
- Khi đề xuất mở rộng, nói rõ đâu là phương pháp của bài gốc và đâu là ý tưởng mới.
- Dùng các thí nghiệm repo làm công cụ hỗ trợ; không mặc định người dùng đang
  yêu cầu tối ưu/sửa DMPC.
- Không gọi ADMM/DCBF là thành phần đã có của bài gốc hoặc simulator.
- Giữ phân biệt lệnh vận tốc outer, vận tốc thực và đầu vào plant.
- Không tuyên bố đã có tránh vật cản động hoặc bảo đảm an toàn khi mới trao đổi ý tưởng.
- Chưa có yêu cầu thay đổi mã mô phỏng; thay đổi hiện tại chỉ là tạo file context này.

## 8. Tài liệu repo đã đọc trong phiên

- `README.md`
- `docs/ARCHITECTURE.md`
- `docs/PAPER_CONTEXT.md`
- `docs/PAPER_VB_IMPLEMENTATION.md`
- `docs/DMPC_IMPLEMENTATION.md`
- `docs/DMPC_FIXES.md`
- `docs/DMPC_AUDIT.md`
- `docs/DMPC_EXPERIMENT_PLAN.md`
- `docs/RELATED_METHODS.md`
- `configs/experiments/README.md`
- `experiments/README.md`
- `drone_bench/simulation/README.md`
- `drone_bench/algorithms/controllers/README.md`
- `presentations/project3_dmpc_admm_dcbf/README.md`

Khi cần biết kết quả chạy chính xác của thí nghiệm phụ, xem artifacts và resolved
config tương ứng; không coi các con số tóm tắt trong context này là nguồn thay
thế cho dữ liệu gốc.
