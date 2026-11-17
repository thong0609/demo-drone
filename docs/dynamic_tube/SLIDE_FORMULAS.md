# Giải thích công thức trên slide 6–10 của báo cáo horizon

Tài liệu này nói về **số trang slide trong PowerPoint**, tính cả trang mở đầu
HUST và trang bìa; không phải phương trình (6)–(10) của bài báo gốc.

Bản trình chiếu: [Project-III-Tube-dong-Horizon-bao-cao.pptx](../../presentations/project3_dynamic_tube_horizon/Project-III-Tube-dong-Horizon-bao-cao.pptx).

Mạch tính xuyên suốt:

```text
Slide 6: dự báo vật cản → tạo vùng quét
Slide 7: tính khoảng cách từ điểm tới vùng quét
Slide 9: dùng khoảng cách để tìm đường bằng enhanced RRT*
Slide 10: làm trơn đường và tạo bán kính tube
Slide 8: kiểm tra cả tube trước khi chấp nhận
```

Slide 8 được trình bày trước slide 9–10 để giải thích yêu cầu hình học. Trong
pipeline thực hành, phải kiểm tra lại sau khi sinh tube.

## Các ký hiệu dễ nhầm

| Ký hiệu | Ý nghĩa |
|---|---|
| tau / τ | Thời điểm hiện tại đang lập kế hoạch |
| h | Khoảng thời gian tính từ hiện tại tới một thời điểm tương lai |
| H | Tổng khoảng thời gian nhìn trước, đơn vị giây |
| s | Tham số vị trí dọc đường tâm tube; không mặc định là thời gian thực |
| t trong minimum snap | Tham số dùng xây đa thức từng đoạn, có thời lượng đoạn |
| c_j | Tâm vật cản thứ j |
| p | Một điểm đang cần kiểm tra; có thể là điểm mẫu của cạnh RRT* |
| p_b(s) | Điểm trên đường tâm tube |
| R_j | Bán kính vật cản, khác bán kính tube |
| r_b(s) | Bán kính tiết diện tube |
| m | Đệm đã thêm vào vật cản khi tạo capsule |
| m_t | Khoảng hở yêu cầu thêm giữa tube và vùng cấm đã có đệm |

Vị trí, khoảng cách và bán kính dùng mét; vận tốc dùng m/s. Trong các công thức
vector, norm là độ dài Euclidean, còn dấu T là chuyển vị để tính tích vô hướng.

## Slide 6 — Dự báo chuyển động và gom thành vùng quét

### 1. Dự báo tâm vật cản

$$
c_j(\tau+h)=c_j(\tau)+v_jh,\qquad 0\leq h\leq H.
$$

Đọc bằng lời: **vị trí tương lai bằng vị trí hiện tại cộng quãng dịch chuyển**.

- Biết tâm hiện tại và vector vận tốc.
- Giả sử vận tốc không thay đổi trong khoảng dự báo.
- Nhân vận tốc với thời gian để được vector dịch chuyển, rồi cộng vào tâm.

Ví dụ ở thời điểm hiện tại, tâm là `(100,80,50)` m, vận tốc `(0,-10,0)` m/s.
Vật đi theo hướng âm Y, không đổi X và Z. Nếu H=3 s:

| Nhìn trước | Tâm dự báo (m) |
|---|---|
| 0 s | (100,80,50) |
| 1 s | (100,70,50) |
| 2 s | (100,60,50) |
| 3 s | (100,50,50) |

H=3 không có nghĩa chỉ kiểm tra đúng vị trí ở giây thứ ba. Công thức tiếp theo
gom mọi vị trí trong cả khoảng 0–3 s.

### 2. Vùng quét

$$
\mathcal S_j(H)=\bigcup_{0\leq h\leq H}
B\big(c_j(\tau)+v_jh,\ R_j+m\big).
$$

Khối cầu được định nghĩa bởi:

$$
B(c,R)=\{x\in\mathbb R^3:\lVert x-c\rVert\leq R\}.
$$

Đọc bằng lời: tại mỗi thời điểm tương lai, đặt một quả cầu có bán kính bằng
**bán kính vật cản + đệm**. Lấy hợp tất cả các quả cầu đó thành một vùng cấm.
Ký hiệu hợp không phải chỉ lấy hai quả cầu ở đầu và cuối; nó bao cả phần giữa.

Với ví dụ trên, nếu R_j=4 m và m=1 m, vùng cấm là capsule dài theo đoạn từ
`(100,80,50)` tới `(100,50,50)`, có bán kính 5 m. Có thể hình dung kéo một quả
cầu bán kính 5 m dọc toàn bộ đoạn đó.

Planner coi cả capsule là vật cản đứng yên trong lần tính này. Nó không tận dụng
việc một chỗ trong capsule có thể đang trống vào một thời điểm nào đó. Đó là
nguồn gốc của tính bảo thủ trong hướng horizon.

**Giới hạn:** công thức đang giả sử cầu bán kính cố định, vận tốc không đổi.
Nếu vật cản đổi hướng, tăng tốc hoặc quan sát sai thì capsule này chưa tự bao
được mọi chuyển động thật. Đệm m hiện là lựa chọn thử nghiệm, không phải bound
bất định đã chứng minh. Công thức không thực hiện nhận diện vật cản.

## Slide 7 — Tính khoảng cách từ điểm tới capsule

### Nguồn của công thức tìm điểm gần nhất

Đây là phép chiếu điểm lên đoạn thẳng chuẩn trong hình học tính toán, không
phải công thức mới do dự án đề xuất và không nên gán cho bài Curve Virtual Tube.

- David Eberly, *Distance Between Point and Line, Ray, or Line Segment*,
  Geometric Tools; tạo ngày 1999-03-02, sửa ngày 2008-03-01, phần Discussion,
  trang 2: [PDF nguồn](https://www.geometrictools.com/Documentation/DistancePointLine.pdf#page=2).
  Tài liệu viết hướng đoạn là M, đầu đoạn là B; đổi B=a và M=b-a sẽ thu được
  phân số dùng trên slide. Ba trường hợp tham số trước/giữa/sau đoạn tương
  đương phép clip vào [0,1].
- Stanford CS248B, Fall 2022, Karen Liu và Doug James, Lecture 2, trang PDF 10,
  *Closest-point projection onto a line segment*: trình bày phép chiếu và
  clamp tọa độ đoạn về [0,1].
  [Slide giảng dạy](https://web.stanford.edu/class/cs248b/cgi-bin/autumn22content/lectures/02_foundations/02_foundations_slides.pdf#page=10).

Có thể tự suy ra bằng bài toán tối thiểu khoảng cách bình phương:

$$
\min_{0\leq\alpha\leq1}\ \lVert p-a-\alpha(b-a)\rVert^2.
$$

Với a khác b, nghiệm không ràng buộc là tích vô hướng chia bình phương độ dài
đoạn. Vì mục tiêu là quadratic lồi theo alpha, nếu nghiệm nằm ngoài [0,1] thì
lấy đầu mút gần nhất; đó chính là clip. Nếu a=b, xử lý riêng như mô tả bên dưới.

Phần áp dụng của dự án là dùng đoạn chuyển động dự báo làm trục capsule, rồi
trừ bán kính để truy vấn vùng quét cho planner. Không coi phép chiếu chuẩn này
là đóng góp mới, cũng không coi có công thức khoảng cách là đã chứng minh hệ
thống tránh được mọi vật cản động.

### 1. Hai đầu đoạn dự báo

$$
a=c_j(\tau),\qquad b=a+v_jH.
$$

a là tâm hiện tại, b là tâm cuối horizon. Ta cần biết điểm p cách đoạn a–b bao
xa, rồi trừ bán kính capsule.

### 2. Tìm điểm gần nhất trên đoạn

$$
\alpha=\operatorname{clip}_{[0,1]}
\left(\frac{(p-a)^T(b-a)}{\lVert b-a\rVert^2}\right),
\qquad q=a+\alpha(b-a).
$$

Phân số là phép chiếu p lên **đường thẳng** đi qua a,b. Nó cho biết điểm chiếu
nằm ở tỷ lệ nào tính từ a tới b:

- alpha=0: tại a.
- alpha=0.5: giữa đoạn.
- alpha=1: tại b.
- alpha ngoài [0,1]: điểm chiếu nằm ngoài đoạn.

`clip` kéo giá trị về [0,1]: nhỏ hơn 0 lấy 0; lớn hơn 1 lấy 1; ở giữa giữ nguyên.
Nhờ đó q là điểm gần nhất trên **đoạn hữu hạn**, không phải trên đường thẳng kéo dài.

Vẫn dùng a=`(100,80,50)`, b=`(100,50,50)`, chọn p=`(110,65,50)`:

$$
b-a=(0,-30,0),\qquad p-a=(10,-15,0).
$$

$$
\alpha=\frac{450}{900}=0.5,\qquad q=(100,65,50).
$$

Điểm p cách q đúng 10 m.

**Trường hợp đặc biệt:** nếu a=b, ví dụ vật cản đứng yên, mẫu số bằng 0. Không
được dùng phép chia trên; đặt q=a và tính khoảng cách tới một quả cầu.

### 3. Trừ bán kính capsule

$$
d_j(p)=\lVert p-q\rVert-(R_j+m).
$$

Với khoảng cách tới đoạn 10 m và bán kính capsule 5 m:

$$
d_j(p)=10-5=5\ \mathrm m.
$$

| Giá trị | Cách hiểu |
|---|---|
| d_j(p)>0 | Điểm nằm ngoài capsule, còn khoảng hở |
| d_j(p)=0 | Điểm nằm trên bề mặt capsule |
| d_j(p)<0 | Điểm nằm trong capsule |

Khi có nhiều vật cản, code lấy giá trị nhỏ nhất của khoảng hở với các capsule,
vật cản tĩnh và biên world. Đây là khoảng hở Euclidean trong vùng trống; không
khẳng định là signed distance chính xác bên trong mọi hợp vật cản chồng lấn.

**Dùng ở đâu?** Enhanced RRT* dùng truy vấn này để kiểm tra cạnh và tính chi phí.
Bước sinh bán kính và bước chứng nhận tube cũng dùng cùng hình học snapshot.
Xem `drone_bench/environment/swept.py`.

## Slide 8 — Kiểm tra tube, không chỉ kiểm tra đường tâm

### 1. Điều kiện cho một vị trí trên tube

$$
d\big(p_b(s)\big)-r_b(s)\geq m_t.
$$

Đọc bằng lời: **khoảng hở từ tâm tới vùng cấm phải đủ cho bán kính tube và phần
dự phòng thêm**.

Ví dụ d(p_b)=5 m từ slide 7, bán kính tube 3 m và m_t=2 m:

$$
5-3=2\geq2.
$$

Điều kiện đạt. Nhưng nếu bán kính tube là 4 m thì còn 1 m, chưa đạt yêu cầu 2 m,
dù chính đường tâm vẫn nằm ngoài vật cản.

Điều kiện này dùng quả cầu bán kính r_b quanh tâm để bao tiết diện đĩa. Vì quả
cầu chứa đĩa, đây là **điều kiện đủ bảo thủ**. Không đạt phép kiểm tra bao cầu
chưa tự chứng minh đĩa tube thực sự chạm vật cản.

Phải đạt dọc toàn đường, không chỉ tại một vài điểm lấy mẫu. Vì đường tâm là
đa thức liên tục, code có thêm cách chứng nhận cả đoạn.

### 2. Chứng nhận cả đoạn bằng một miền bao

$$
d(q)-E-r_{\mathrm{upper}}\geq m_t.
$$

**q trong slide này khác q của slide 7.** Ở đây q là tâm một miền bao đường cong;
nó không phải điểm chiếu gần nhất trên capsule.

| Ký hiệu | Vai trò |
|---|---|
| q | Tâm dùng để bao một đoạn đường cong |
| E | Bán kính bảo thủ: mọi điểm đường tâm của đoạn cách q không quá E |
| r_upper | Cận trên bán kính tube trong cả đoạn |
| d(q) | Khoảng hở từ tâm bao tới vùng cấm |

Với đa thức Bernstein, đường cong nằm trong convex hull của các control points.
Control points là các hệ số hình học điều khiển đường cong; không đồng nhất
với các waypoint mà RRT* tìm được. Từ chúng, code tìm một miền bao có tâm q và
bán kính E. Cận trên bán kính tube lấy từ control points của đa thức bán kính.

Lý do được phép trừ E: hàm khoảng hở là 1-Lipschitz, tức di chuyển một đoạn x
không thể làm khoảng hở giảm quá x. Vì vậy:

$$
\lVert p_b(s)-q\rVert\leq E
\quad\Rightarrow\quad
d(p_b(s))\geq d(q)-E.
$$

Trừ tiếp r_upper và so với m_t sẽ kiểm tra bảo thủ toàn thể tích tube được bao.

Ví dụ d(q)=15 m, E=4 m, r_upper=8 m, m_t=2 m:

$$
15-4-8=3\geq2.
$$

Cả đoạn đạt chứng nhận. Nếu miền bao quá rộng nên chưa đạt, code chia đôi đoạn
đa thức để các miền bao chặt hơn rồi kiểm tra lại. Chia tới giới hạn vẫn không
đạt thì từ chối candidate, không mặc định nó an toàn.

**Giới hạn:** đây là chứng nhận hình học với vùng cấm dự báo. Nó chưa chứng minh
tube không tự chồng lấn, UAV thực bám được, hoặc tube còn hợp lệ sau trễ tính.
Đệm m đã có trong capsule; m_t là lớp khoảng hở riêng. Không cộng nhầm hai đại
lượng này thành hai lần bán kính drone.

## Slide 9 — Enhanced RRT* chọn đường bằng chi phí nào?

### 1. Mixed cost

$$
J(a,b)=\lambda_1\lVert b-a\rVert
+\lambda_2\sum_{k=1}^{n}P(d_k).
$$

a,b ở slide này là **hai nút của cây RRT***, không nhất thiết là hai đầu capsule.

- Thành phần đầu phạt độ dài đoạn nối: đường dài hơn thường tốn hơn.
- Thành phần sau phạt việc đi sát vật cản, tính qua các điểm mẫu trên đoạn nối.
- d_k là khoảng hở ở điểm mẫu thứ k, lấy từ world đã có các capsule.
- lambda_1 và lambda_2 điều chỉnh đánh đổi giữa độ dài và khoảng trống.

Nói đơn giản: thuật toán không chỉ tìm cách tới đích ngắn, nó còn muốn có đủ
không gian xung quanh để tạo tube.

### 2. Penalty gần vật cản

Slide ghi nhánh khi khoảng hở nhỏ; viết đầy đủ theo (15)–(16) của bài báo là:

$$
P(d)=
\begin{cases}
\displaystyle\frac{(r_e-d)^3}{d^3+\kappa}, & 0\leq d<r_e,\\[6pt]
0,&d\geq r_e.
\end{cases}
$$

r_e là bán kính tube tối đa; kappa dương làm mẫu số không bằng 0 khi d=0.
Ví dụ minh họa chọn r_e=20 m, kappa=1 theo thang số đang dùng:

| Khoảng hở d | Penalty xấp xỉ |
|---|---:|
| 5 m | 26.79 |
| 15 m | 0.037 |
| 20 m trở lên | 0 |

Đi sát vật cản bị phạt mạnh hơn. Các trọng số/kappa phải phù hợp thang đo và
đơn vị; penalty không được hiểu như khoảng cách tính bằng mét.

Chi phí thấp **không thay thế điều kiện không va chạm**. Với d<0, đoạn đi vào
vùng cấm phải bị loại; không kéo dài hàm trên sang vùng âm để chấp nhận cạnh.
Code còn kiểm tra khoảng giữa các mẫu bằng dự phòng Lipschitz.

Số lượng/cách đặt mẫu ảnh hưởng tổng penalty, nên khi so các horizon phải giữ
nguyên cách lấy mẫu và các trọng số.

### 3. Phần cải tiến Visual

Ngoài mixed cost, thuật toán thử nối nút mới tới các nút tổ tiên, nếu kết nối
không va chạm và làm tổng chi phí tốt hơn. Nó giúp bỏ bớt đường vòng trong cây.
Đó là thao tác của enhanced RRT*, khác với minimum snap làm trơn sau khi đã có
waypoint. Nguồn: Algorithms 2–3, phương trình (15)–(16) của bài báo chính.

## Slide 10 — Minimum snap tìm đường trơn như thế nào?

### 1. Biến cần tìm là hệ số đường cong

Sau RRT*, ta có các waypoint nối bằng đoạn thẳng. Minimum snap tìm đa thức từng
đoạn để biểu diễn đường cong trơn hơn. Ví dụ một chiều:

$$
\sigma_k(t)=a_{k,0}+a_{k,1}t+\cdots+a_{k,n}t^n.
$$

Gom các hệ số của mọi đoạn thành vector w. Trong code, dùng cơ sở Bernstein bậc
7 thay cho cơ sở lũy thừa viết minh họa ở trên. Vẫn là tìm các hệ số đa thức.

sigma có thể biểu diễn một tọa độ x/y/z; khi sinh đường bán kính, đại lượng cần
fit là bán kính từ thông tin clearance. Không phải tối ưu trực tiếp vị trí mọi
drone hoặc đầu vào actuator trong công thức này.

### 2. Hai thành phần của mục tiêu

Viết đầy đủ khoảng tích phân của công thức trên slide:

$$
\min_w\quad
\int_{t_0}^{t_f}|\sigma^{(4)}(t)|^2\,dt
+\gamma\int_{t_0}^{t_f}|\sigma(t)-\sigma_b(t)|^2\,dt.
$$

| Thành phần | Ý nghĩa |
|---|---|
| sigma^(4) | Đạo hàm bậc bốn, gọi là snap |
| Tích phân bình phương snap | Giảm mức snap tổng cộng dọc các đoạn |
| sigma_b(t) | Đường tham chiếu từ benchmark/waypoint |
| Tích phân bình phương sai lệch | Phạt đường mới lệch xa đường tham chiếu |
| gamma | Trọng số cho việc giữ gần đường tham chiếu |

“Tích phân” ở đây có thể hiểu là cộng liên tục chi phí trên cả khoảng. Bình
phương giúp sai lệch lớn bị phạt mạnh và không để các giá trị trái dấu triệt nhau.

Không có hạng phạt lệch đường thì việc ưu tiên độ trơn có thể đưa đường đi xa
hơn mong muốn. Ngược lại, tăng gamma không tự bảo đảm đường tốt nhất theo mọi
tiêu chí. Các ràng buộc và thời lượng từng đoạn cũng ảnh hưởng nghiệm.

### 3. Ràng buộc

$$
A_{\mathrm{eq}}w=b_{\mathrm{eq}},\qquad
A_{\mathrm{ieq}}w\leq b_{\mathrm{ieq}}.
$$

Hai biểu thức là cách viết gọn nhiều điều kiện tuyến tính theo hệ số:

- Điều kiện bằng: vị trí và đạo hàm đầu/cuối; nối các đoạn liên tục tới đạo hàm
  bậc hai. Code hiện tại còn cố định vị trí các waypoint.
- Điều kiện không vượt: giữ đường trong corridor quanh đường benchmark.

A là ma trận hệ số đã được lập từ basis/thời lượng; b chứa giá trị yêu cầu.
Solver tìm w để giảm mục tiêu đồng thời thỏa các điều kiện này. Dạng mục tiêu
bình phương và ràng buộc tuyến tính cho bài toán QP.

Trong bản hiện thực, corridor được kiểm tra bảo thủ bằng control points để
giới hạn cả đoạn đa thức, không chỉ một vài điểm trên hình vẽ.

### 4. Vì sao “minimum snap” vẫn có thể xuất hiện đoạn vòng?

Công thức này tối ưu snap và độ lệch tham chiếu, **không trực tiếp tối ưu mọi
đặc tính hình dạng**, chẳng hạn độ dài nhỏ nhất, không tự chồng lấn hoặc một
cận độ cong cụ thể. Vì vậy đạt nghiệm QP chưa đủ kết luận đường đã tốt về mọi mặt.

Ca xoáy trong demo xuất hiện sau làm trơn waypoint. Đổi thời lượng đoạn giúp
giảm đoạn vòng ở ca đó nhưng tăng độ cong cực đại lấy mẫu; chưa được gọi là
giải quyết mọi vấn đề hình dạng. Đây là hạn chế của bản hiện thực/cấu hình đang
chạy, chưa phải bằng chứng bài báo gốc có cùng lỗi.

Đặc biệt, t trong công thức xây đường không ép mọi UAV phải tới cùng tiết diện
đúng cùng thời điểm. Chưa có lịch di chuyển của đàn được tối ưu ở đây. Vì vậy
minimum snap này chưa biến hướng horizon thành planner không gian–thời gian.

## Khi giải thích trên lớp có thể nói ngắn như sau

| Slide | Câu giải thích chính |
|---|---|
| 6 | Tôi dự báo vật cản đi đâu trong H giây rồi gom cả thể tích nó đi qua thành vùng cấm. |
| 7 | Tôi tính điểm gần nhất trên đoạn chuyển động rồi trừ bán kính cầu để biết khoảng hở tới capsule. |
| 8 | Tôi yêu cầu khoảng hở đủ cho cả bán kính tube và chứng nhận cả đoạn, không chỉ đường tâm. |
| 9 | Enhanced RRT* tìm tuyến vừa xét độ dài vừa tránh đi quá sát vùng cấm. |
| 10 | Minimum snap làm trơn tuyến bằng cách tối ưu hệ số đa thức; sau đó vẫn phải kiểm tra lại tube. |

Nguồn đối chiếu: [PLANNER.md](PLANNER.md),
[giải thích planner bài gốc](../paper/TUBE_PLANNING.md), Algorithms 2–3 và
phương trình (15)–(21) trong PDF chính. Công thức capsule/chứng nhận là phần
triển khai mở rộng của dự án; không gán chúng thành công thức nguyên gốc của bài.
