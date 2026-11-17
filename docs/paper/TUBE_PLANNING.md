# Giải thích thuật toán lập curve virtual tube trong bài báo

Tài liệu này giải thích Section III, Algorithms 1–3 và các phương trình (15)–(21)
của bài *Curve Virtual Tube Planning and Distributed Control for Swarm UAVs With
Complex Constraints and Disturbances*, Yuan và cộng sự, IEEE TIE, 2026.

Đây là giải thích phương pháp của bài báo. Các nhận xét về giới hạn và khác biệt
với implementation trong repo được ghi riêng ở cuối tài liệu.

## 1. Đầu ra của thuật toán là gì?

Lập tube gồm hai việc: tìm đường đi giữa các vật cản, rồi tạo đường tâm trơn và
bán kính thay đổi dọc đường.

Đầu ra là đường tâm:

$$
p_b(t)=
\begin{bmatrix}
x_b(t)\\
y_b(t)\\
z_b(t)
\end{bmatrix}
$$

và đường bán kính:

$$
r_b(t).
$$

Ở mỗi giá trị tham số, đường tâm xác định tâm tiết diện; bán kính xác định độ rộng
của tiết diện đó. Một đường cong duy nhất chưa đủ để mô tả cả tube.

Luồng xử lý của Algorithm 1:

```text
Bản đồ + điểm đầu + điểm đích
               |
               v
         Tính bản đồ SDF
               |
               v
   Đặt các giới hạn của tube
               |
               v
    Enhanced RRT* tìm đường
               |
               v
 Các waypoint và khoảng cách vật cản
               |
               v
      Minimum-snap generation
               |
               v
  Đường tâm trơn + bán kính trơn
               |
               v
      Curve virtual tube
```

## 2. SDF: biết mỗi vị trí cách vật cản bao xa

Nếu chỉ biết một điểm có va chạm hay không, planner có thể tìm một đường không
chạm vật cản nhưng đi rất sát tường. Đường đó có thể phù hợp cho một điểm chuyển
động, nhưng khó đặt một tube đủ rộng quanh nó.

SDF bổ sung thông tin khoảng cách tới vật cản. Chẳng hạn:

- Điểm A cách vật cản gần nhất 2 m.
- Điểm B cách vật cản gần nhất 20 m.

Cả hai đều nằm trong vùng trống, nhưng B thuận lợi hơn để đặt một tiết diện tube
lớn. Thông tin này được dùng để chọn tuyến đường và xây dựng bán kính.

Theo mô tả Section III, SDF cho phép truy vấn thông tin khoảng cách và gradient.
Trong mixed cost của enhanced RRT*, đại lượng xuất hiện trực tiếp là khoảng cách
đến vật cản gần nhất tại các điểm lấy mẫu.

## 3. Enhanced RRT*: chọn tuyến vừa ngắn vừa có khoảng trống

### 3.1. Cơ chế xây cây

RRT* xây một cây từ điểm xuất phát bằng cách lấy mẫu điểm trong không gian, nối
những đoạn không va chạm và cập nhật các kết nối để giảm chi phí.

Algorithm 2 có các thao tác chính:

1. Khởi tạo cây với nút gốc là điểm xuất phát.
2. Lấy mẫu một điểm ngẫu nhiên.
3. Tìm nút gần nhất và tiến một bước để tạo nút mới.
4. Kiểm tra đoạn nối có va chạm không.
5. Tìm các nút lân cận và chọn parent có chi phí phù hợp.
6. Dùng `Visual` để thử cải thiện parent.
7. Rewire các kết nối lân cận nếu đi qua nút mới làm giảm chi phí.
8. Ghi nhận các nút đạt vùng đích.

Hai cải tiến tác giả nhấn mạnh là mixed cost và chọn lại parent bằng `Visual`.

### 3.2. Cải tiến thứ nhất: mixed cost

Phương trình (15) kết hợp độ dài đoạn nối và chi phí gần vật cản. Đổi tên một số
biến để dễ đọc, ta có:

$$
J(a,b)=\lambda_1\lVert b-a\rVert
      +\lambda_2\sum_{k=1}^{n}P(d_k).
$$

Trong đó:

| Ký hiệu | Ý nghĩa |
|---|---|
| a, b | Hai đầu đoạn nối |
| n | Số điểm lấy mẫu trên đoạn, không phải số UAV |
| d_k | Khoảng cách từ điểm mẫu thứ k đến vật cản gần nhất |
| P(d_k) | Chi phí phạt khi đi gần vật cản |
| lambda_1 | Trọng số của độ dài |
| lambda_2 | Trọng số của penalty vật cản |

Hàm phạt ở (16) là:

$$
P(d)=
\begin{cases}
\displaystyle\frac{(r_e-d)^3}{d^3+\kappa}, & 0\leq d<r_e,\\[6pt]
0, & d\geq r_e,
\end{cases}
$$

với bán kính tube tối đa là r_e và hằng số kappa dương.

Ý nghĩa:

- Đoạn càng dài thì thành phần độ dài càng lớn.
- Các điểm trên đoạn càng gần vật cản thì penalty càng lớn.
- Khi khoảng cách đã ít nhất bằng bán kính tube tối đa, penalty bằng 0.

Ví dụ minh họa: một tuyến ngắn đi qua khe hẹp và một tuyến dài hơn đi qua vùng
rộng. Tùy trọng số, planner có thể chọn tuyến dài hơn để tạo tube thuận lợi hơn.
Đây là ví dụ giải thích, không phải một kết quả số được trích từ bài.

Penalty này là ưu tiên trong hàm mục tiêu, không tự thay thế kiểm tra va chạm hay
một ràng buộc cứng bảo đảm mọi chỗ đều đủ rộng cho đàn.

### 3.3. Cải tiến thứ hai: chọn lại parent bằng Visual

Giả sử cây đang có chuỗi:

```text
A → B → C → D
```

Khi xét D, thuật toán thử nối D với một ancestor xa hơn, chẳng hạn B hoặc A.
Một kết nối được dùng để cải thiện parent khi đoạn nối không va chạm và tổng
mixed cost tốt hơn. Algorithm 3 kiểm tra lần lượt theo chuỗi ancestor.

Ví dụ, nếu nối B tới D hợp lệ và rẻ hơn đi qua C, cấu trúc có thể trở thành:

```text
A → B → D
```

Tác dụng là loại bớt các đoạn vòng không cần thiết. Việc bỏ nút không có nghĩa
được phép xuyên qua vật cản: vẫn có điều kiện `CollisionFree`.

Sau bước tìm đường, kết quả dùng cho bước tiếp theo là một đường gấp khúc qua
các waypoint. Nó chưa phải đường cong tube cuối cùng.

## 4. Minimum snap: biến đường gấp khúc thành đường cong trơn

### 4.1. Vì sao cần làm trơn?

Nối waypoint bằng đoạn thẳng tạo các góc gãy. Tác giả dùng đa thức từng đoạn để
tạo đường cong trơn hơn.

Với một đại lượng một chiều, chẳng hạn tọa độ x, mỗi đoạn có dạng:

$$
\sigma_j(t)=a_{j,0}+a_{j,1}t+\cdots+a_{j,n}t^n.
$$

Các hệ số đa thức là biến cần tìm. Bài trình bày bài toán một chiều rồi áp dụng
cho các chiều tương ứng. Không mặc định bậc đa thức cụ thể của implementation
trong repo là một thông số đã được tác giả ấn định.

### 4.2. Hàm mục tiêu

Phương trình (20):

$$
\min_w f(w)
=\int_{t_0}^{t_f}\left|\sigma^{(4)}(t)\right|^2\,dt
+\gamma\int_{t_0}^{t_f}\left|\sigma(t)-\sigma_b(t)\right|^2\,dt.
$$

Trong đó w gom các hệ số của các đoạn đa thức; sigma_b là đường benchmark dùng
để tham chiếu.

Hai thành phần có vai trò khác nhau:

| Thành phần | Mục đích |
|---|---|
| Tích phân bình phương đạo hàm bậc bốn | Giảm snap |
| Tích phân bình phương sai lệch với benchmark | Giữ đường mới gần đường ban đầu |

Chuỗi đạo hàm theo tham số thời gian thường được gọi là:

```text
Vị trí → vận tốc → gia tốc → jerk → snap
```

Gamma điều chỉnh mức ưu tiên bám đường benchmark. Tăng gamma làm tăng chi phí của
việc lệch khỏi benchmark, nhưng không có nghĩa mọi điểm bắt buộc trùng benchmark.

Giảm snap không tự bảo đảm tốc độ hay đầu vào thực của UAV thỏa giới hạn: chuyển
động thực còn phụ thuộc controller và plant.

### 4.3. Các ràng buộc

**Điều kiện biên — phương trình (17):**

$$
\begin{aligned}
\sigma(t_0)&=p_{\mathrm{start}},&
\dot\sigma(t_0)&=v_{\mathrm{start}},&
\ddot\sigma(t_0)&=a_{\mathrm{start}},\\
\sigma(t_f)&=p_{\mathrm{goal}},&
\dot\sigma(t_f)&=v_{\mathrm{goal}},&
\ddot\sigma(t_f)&=a_{\mathrm{goal}}.
\end{aligned}
$$

**Nối liên tục các đoạn — phương trình (18):**

$$
\begin{aligned}
\sigma_{\mathrm{before}}(t_i)&=\sigma_{\mathrm{after}}(t_i),\\
\dot\sigma_{\mathrm{before}}(t_i)&=\dot\sigma_{\mathrm{after}}(t_i),\\
\ddot\sigma_{\mathrm{before}}(t_i)&=\ddot\sigma_{\mathrm{after}}(t_i).
\end{aligned}
$$

Như vậy các đoạn nối liên tục đến đạo hàm bậc hai. Không nên tự diễn giải rằng
phương trình này yêu cầu liên tục mọi đạo hàm đến bậc bốn.

**Corridor — phương trình (19):** giới hạn đường mới trong một hành lang quanh
đường benchmark. Về ý nghĩa, mỗi chiều có một khoảng cho phép quanh giá trị
benchmark. Corridor hạn chế việc đường làm trơn lệch quá xa tuyến ban đầu.

Cần phân biệt ràng buộc tại các điểm kiểm tra và một chứng nhận cho toàn bộ đoạn
đa thức. Không chỉ từ mô tả corridor mà mặc định toàn bộ tube đã được chứng nhận
không va chạm ở mọi vị trí.

### 4.4. Đưa về bài toán QP

Phương trình (21) viết bài toán dưới dạng:

$$
\begin{aligned}
\min_w\quad &f(w)\\
\text{subject to}\quad&A_{\mathrm{eq}}w=b_{\mathrm{eq}},\\
&A_{\mathrm{ieq}}w\leq b_{\mathrm{ieq}}.
\end{aligned}
$$

Do đường cong và đạo hàm phụ thuộc tuyến tính vào các hệ số, còn mục tiêu là tích
phân các biểu thức bình phương, bài toán có thể chuyển thành quy hoạch toàn
phương. Tác giả nêu giải bằng phương pháp interior-point và dẫn tài liệu [24]
cho quy trình chuyển đổi chi tiết.

### 4.5. Hiểu tham số t như thế nào?

Đường cong được xây dựng theo tham số t với các nút chia đoạn. Không vì thế mà
mọi UAV phải tới cùng một tiết diện đúng cùng một thời điểm. Outer loop sau đó
điều phối chuyển động của từng UAV và tạo guidance curve tương ứng.

## 5. Bán kính tube được tạo như thế nào?

Theo mô tả Section III:

- Đường tâm được sinh dựa trên benchmark path.
- Đường bán kính được sinh từ thông tin khoảng cách SDF tương ứng.
- Các giới hạn bán kính tối thiểu và tối đa được đặt trong bước cấu hình tube.
- Phương pháp sinh đường cong được dùng để tạo các đường mô tả tube.

Trực giác:

```text
Vùng rộng → có thể mở bán kính
Gần vật cản → cần thu bán kính
Ra khỏi chỗ hẹp → có thể mở lại
```

Phần xây dựng bán kính được bài trình bày ít chi tiết hơn mixed cost và QP. Không
nên tự gán công thức “bán kính luôn bằng đúng SDF” cho tác giả. Muốn tái hiện
chính xác cần kiểm tra thêm các thông số và chi tiết chưa được mô tả đầy đủ.

## 6. Ghép đường tâm và bán kính thành tube

Tại mỗi điểm đường tâm, lấy một đĩa tròn có tâm là điểm đó và mặt phẳng vuông góc
với tiếp tuyến đường tâm. Viết theo tham số để giải thích (12)–(13):

$$
C(t)=\left\{q\in\mathbb R^3:
\lVert q-p_b(t)\rVert\leq r_b(t),\quad
u_t(t)^T\big(q-p_b(t)\big)=0\right\}.
$$

Ở đây u_t là vector tiếp tuyến đơn vị, tại nơi tiếp tuyến được xác định.

Tube là hợp của các tiết diện:

$$
\mathcal T=\bigcup_{t\in[t_0,t_f]}C(t).
$$

Hình dung: xâu nhiều đĩa tròn có kích thước thay đổi dọc một đường cong. Đây là
hình học tube; guidance curve của từng UAV được tạo ở bước outer control sau đó.

## 7. Những giới hạn cần hiểu đúng

### 7.1. Số lượng drone chưa trực tiếp quyết định hình dạng tube

Trong công thức lập kế hoạch được trình bày, tube chịu tác động trực tiếp từ bản
đồ, vật cản, điểm đầu–đích, mixed cost, corridor và điều kiện làm trơn. Chưa có
ràng buộc sức chứa đàn dùng số UAV để tự quyết định độ rộng tube.

Cùng một tube có thể phục vụ nhiều drone hơn nếu chúng đi lần lượt, nhưng khi
đó cần xét thêm thời gian đi qua, mật độ, khoảng cách an toàn và giới hạn độ dài
đàn do yêu cầu cohesion. Tube tồn tại không tự chứng minh mọi bố trí đàn đều đi
qua được mà thỏa mọi ràng buộc.

### 7.2. Làm trơn chưa tự chứng nhận toàn tube không va chạm

Đường gấp khúc ban đầu không va chạm chưa đủ để kết luận đường tâm sau làm trơn
và toàn bộ tiết diện tube đều không va chạm. Bán kính cũng có thể biến thiên giữa
các điểm đã kiểm tra. Cần kiểm tra hình học phù hợp sau khi sinh tube.

Trong repo, bước co bán kính và chứng nhận swept-ball là biện pháp của
implementation. Không gán chúng thành công thức nguyên gốc của bài báo.

### 7.3. Tube đang dựa trên bản đồ dùng lúc lập kế hoạch

Nếu vật cản động mới bay vào tube, vùng trống dựa trên bản đồ ban đầu có thể
không còn đúng. Pipeline này chưa tự cung cấp một cơ chế phản ứng với viên gạch
bay vào giữa đàn. Cần thiết kế thêm tránh vật cản động hoặc cập nhật/replan tube.

Tác giả đặt sinh tube thời gian thực trong môi trường động vào hướng nghiên cứu
tương lai ở Section VI, trang 3048.

## 8. Nguồn đối chiếu

PDF chính trên máy:

```text
/home/tts/Downloads/1. Curve_Virtual_Tube_Planning_and_Distributed_Control_for_Swarm_UAVs_With_Complex_Constraints_and_Disturbances (1).pdf
```

- Section II-C, (10)–(13): mô hình hình học tube.
- Section III, trang 3040: Algorithm 1, enhanced RRT* và Visual.
- Phương trình (15)–(16): mixed cost và penalty vật cản.
- Section III-B, trang 3041, (17)–(21): sinh đường cong bằng minimum snap.
- Section IV-A, (22): guidance curve của từng UAV.
- Section VI, trang 3048: hướng phát triển trong môi trường động.

Context trao đổi tổng thể nằm trong [CONVERSATION_CONTEXT.md](../research/CONVERSATION_CONTEXT.md).
