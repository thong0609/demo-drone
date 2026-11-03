# Các phương pháp liên quan và kế hoạch so sánh

Ngày khảo sát: 2026-11-03. Đây là danh sách tuyển chọn từ bài báo, trang nhóm tác giả
và kho mã nguồn; chưa cài đặt hay benchmark các phương pháp này. Mức ưu tiên là
đánh giá kỹ thuật cho dự án hiện tại, không phải bảng xếp hạng hiệu năng đã đo.

## 1. Phải xác định phần nào đang được so sánh

- Controller: giữ nguyên file tube; thay trường vector hoặc bộ điều khiển trong.
- Planner tạo tube: giữ map; dùng một controller chung để đánh giá các tube khác nhau.
- Hệ thống lập quỹ đạo đa UAV: giữ map, plant, xuất phát và miền đích; mỗi hệ thống
  có thể tự tạo quỹ đạo. Không bắt buộc tube nếu phương pháp gốc không dùng tube.

Không gọi việc thêm ràng buộc tube vào một thuật toán gốc là tái hiện nguyên bản.
Tên dùng trong báo cáo phải phân biệt original, reimplementation, adaptation và
ablation. Baseline hiện tại là hiện thực trên map tự dựng với tham số giả định;
không đại diện cho toàn bộ hiệu năng công bố của bài gốc.

## 2. Danh sách nên đọc

### A. Curve virtual tube vector field — baseline nền tảng, 2023

Quan Quan, Yan Gao, Chenggang Bai: *Distributed Control for a Robotic Swarm to Pass
through a Curve Virtual Tube*. Robotics and Autonomous Systems, 2023; preprint có
các phiên bản 2021–2022. Ba thành phần chính: tiếp cận đường, tránh robot và giữ
trong tube. Đây là mốc nền để kiểm tra lợi ích của guidance riêng và cohesion trong
bài đang hiện thực; không phải phương pháp mới nhất.

[Bài báo/preprint](https://arxiv.org/abs/2112.01006).

Đề xuất: dùng cùng tube và inner loop khi so vòng ngoài. Tách riêng phiên bản bỏ
cohesion/bỏ actor–critic thành ablation; không gán chúng cho thuật toán gốc khác.

### B. Optimal virtual tube + DMPC — đối thủ gần về cấu trúc

Pengda Mao, Rao Fu, Quan Quan: *Optimal Virtual Tube Planning and Control for Swarm
Robotics*, IJRR 2024. Xây virtual tube từ một tập hữu hạn quỹ đạo tối ưu, dùng biểu
diễn tổ hợp lồi và kết hợp DMPC. Có kho mã của tác giả. Hình học optimal tube không
đồng nhất với tube tròn đang có, nên toàn pipeline cần adapter hoặc đánh giá riêng.

[Bài báo](https://arxiv.org/abs/2304.11407) ·
[Mã nguồn](https://github.com/MorePanda123/optimal-virtual-tube-planning).

Một mở rộng đáng đọc: *An Efficient Real-Time Planning Method for Swarm Robotics
Based on an Optimal Virtual Tube*, preprint nộp 2025, cập nhật 2026. Tập trung giảm
chi phí sinh quỹ đạo qua các hàm affine. Chưa suy diễn tình trạng xuất bản journal
chỉ từ trang arXiv.

[Preprint](https://arxiv.org/abs/2505.01380).

### C. Dynamic tube + DMPC — nghiên cứu gần đây, sát bài toán

Dai, Liao, Chen: *Safe Swarm Navigation in Constrained Environments: A Dynamic
Tube-Based Distributed MPC Approach*, Drones 10(3),177, 5 March 2026. Kết hợp tube
điều chỉnh theo vật cản động và kích hoạt ràng buộc tránh va chạm theo nguy cơ dự báo.

[Bài báo](https://doi.org/10.3390/drones10030177).

Đề xuất: đánh giá bản static tube trước, sau đó mới thêm vật cản động. Phần dynamic
tube sẽ cần mở rộng benchmark. Chưa xác nhận kho mã tác giả trong lần khảo sát này;
không suy ra rằng tác giả không công bố mã.

### D. DMPC-Swarm / MLR-DMPC — phù hợp khi xét lỗi truyền thông

Gräfe và cộng sự: *DMPC-Swarm: distributed model predictive control on nano UAV
swarms*, Autonomous Robots, 2025. Hướng chính là phân tán tính toán và xử lý mất gói,
độ trễ; thực nghiệm tới 16 nano-quadcopters. Có phần mềm/hardware được dẫn từ bài.

[Bài báo](https://link.springer.com/article/10.1007/s10514-025-10211-w) ·
[Mã được bài báo dẫn](https://github.com/Data-Science-in-Mechanical-Engineering/DMPC-Swarm).

Đề xuất: ưu tiên sau khi simulator có model trao đổi thông điệp và timestamps;
chạy hoàn hảo không trễ chưa đánh giá được điểm mạnh chính của công trình.

### E. CBF-QP / HOCBF — bộ lọc an toàn để ghép với controller hiện tại

CBF-QP tìm input gần input danh định, nhưng thỏa bất đẳng thức an toàn. HOCBF xử lý
ràng buộc có relative degree cao, chẳng hạn khoảng cách theo vị trí trong plant
điều khiển bằng lực/gia tốc. Đây là một họ phương pháp nền tảng, không phải một bài
swarm mới nhất hay một thuật toán lập đường hoàn chỉnh.

[CBF-QP, Ames et al., 2017](https://authors.library.caltech.edu/records/jnhr0-1ww05) ·
[HOCBF, Xiao & Belta, TAC 2022](https://calinbelta.com/wp-content/uploads/2023/12/High-Order_Control_Barrier_Functions.pdf).

Đề xuất của dự án (chưa triển khai): giữ guidance hiện tại làm nominal; thêm QP ở
mức lực/gia tốc thực để xử lý tốc độ, khoảng cách UAV và biên tube. Với plant không
chắc chắn phải dùng bound/thiết kế robust phù hợp, kiểm tra trạng thái đầu hợp lệ,
khả thi QP và tác động lấy mẫu. Không tuyên bố bảo đảm cứng chỉ vì có một lớp QP.

### F. AMSwarm — tối ưu quỹ đạo phân tán bằng alternating minimization

Adajania et al., ICRA 2023: *AMSwarm: An Alternating Minimization Approach for Safe
Motion Planning of Quadrotor Swarms in Cluttered Environments*. Biểu diễn polar cho
ràng buộc tránh va chạm và alternating minimization để giữ các bước tối ưu có cấu
trúc QP. Repo gồm cả SCP và ACADO baselines.

[Bài báo](https://arxiv.org/abs/2303.04856) ·
[Mã nguồn](https://github.com/learnsyslab/AMSwarm) ·
[Bản JAX từ cùng tổ chức](https://github.com/learnsyslab/axswarm).

Đề xuất: phù hợp so end-to-end trên cùng map. Nếu thêm tube, ghi là tube-constrained
adaptation. Định nghĩa miền đích và tracker thống nhất trước khi đo hiệu năng.

### G. Primitive-Swarm — ứng viên hiện đại về khả năng mở rộng

Hou et al., IEEE T-RO 2025: *Primitive-Swarm: An Ultra-lightweight and Scalable
Planner for Large-scale Aerial Swarms*. Thư viện motion primitives khả thi động học
được chuẩn bị offline; online chọn chuyển động qua kiểm tra xung đột không–thời
gian. Có mã tác giả. Repo hiện hướng dẫn ROS Noetic/Ubuntu 20.04 và sinh thư viện
bằng TOPP-RA.

[Bài báo](https://arxiv.org/abs/2502.16887) ·
[Mã nguồn](https://github.com/ZJU-FAST-Lab/Primitive-Planner).

Đề xuất: dùng làm đối thủ planning hiện đại, nhất là khi tăng số UAV. Tách chi phí
chuẩn bị thư viện khỏi chi phí online; cũng báo tổng chi phí. Không so trực tiếp
thời gian C++/ROS với Python hiện tại để kết luận chất lượng thuật toán.

### H. EGO-Swarm và Robust MADER — bổ sung khi mở rộng phạm vi

EGO-Swarm dùng local trajectory optimization phân tán, bất đồng bộ; có code công
khai. Phân biệt repo EGO-Swarm với EGO-Planner-v2 gắn với công trình *Swarm of micro
flying robots in the wild*; không trộn tên/phiên bản khi báo cáo.

[EGO-Swarm](https://arxiv.org/abs/2011.04183) ·
[Repo EGO-Swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm) ·
[Repo EGO-Planner-v2](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2).

Robust MADER (2023) hướng vào lập quỹ đạo phân tán bất đồng bộ có độ trễ truyền
thông. Dùng khi benchmark đã có delay/dynamic obstacles. Repo có ROS và Gurobi,
khác đáng kể với môi trường Python chuẩn hiện tại.

[Trang nhóm MIT](https://acl.mit.edu/projects/real-world-multi-agent-trajectory-planning) ·
[Mã nguồn](https://github.com/mit-acl/rmader).

## 3. Bộ so sánh đầu tiên được đề xuất

| ID | Phương pháp | Câu hỏi |
|---|---|---|
| B0 | Hiện thực bài Curve Tube hiện tại | Mốc tham chiếu |
| B1 | Vector field curve tube nền tảng + cùng inner | Guidance/cohesion có ích ra sao? |
| B2 | B0 + CBF/HOCBF filter | Đổi throughput/effort để giảm vi phạm bao nhiêu? |
| B3 | DMPC thích nghi với tube cố định | Dự báo xung đột so với phản ứng tức thời? |
| B4 | AMSwarm hoặc Primitive-Swarm nguyên bản | Hệ thống khác đạt cùng nhiệm vụ trên map ra sao? |

B2 và B3 là thiết kế/adaptation của dự án; chỉ được dùng tên bài cụ thể khi đã
thực hiện đúng thuật toán đó và nêu phần thay đổi. Không gọi một MPC tự viết là
DMPC-Swarm hoặc Dynamic Tube DMPC nguyên bản.

## 4. Giao thức đánh giá đề xuất

1. Controller benchmark: cố định benchmark.json, trạng thái đầu, dynamics, nhiễu,
   giới hạn input, khoảng cách an toàn, truyền thông và miền đích. Khi chỉ thay outer,
   giữ inner giống nhau; thêm phép thử vận tốc lý tưởng để tách ảnh hưởng inner.
2. End-to-end benchmark: giữ map và nhiệm vụ; không ép mọi phương pháp dùng chung
   đường/tube. Nếu tube là luật của nhiệm vụ thì adapter phải được công bố.
3. Khởi đầu N=27; sau đó N=8/64 với quy tắc bố trí hợp lệ được công bố trước. Khi
   đổi mật độ, tách khỏi phép thử chỉ đổi N để tránh đánh đồng hai hiệu ứng.
4. Với phương pháp ngẫu nhiên, chạy nhiều seed được chọn trước, ví dụ 20 seed.
   Tune trên tập validation; dùng cùng ngân sách tune và giữ tập đánh giá riêng.
5. Đo completion rate/time, khoảng cách cặp/biên, vận tốc thực, input, effort proxy
   integral ||u||² dt (không gọi là năng lượng điện), jerk, cohesion và throughput.
6. Đo thời gian tính toán mean/p95/p99, deadline misses, số QP infeasible, bộ nhớ,
   lượng truyền thông. Bao gồm tất cả lần thất bại trong tỷ lệ thành công; báo thời
   gian hoàn thành có điều kiện thành công tách riêng.
7. Báo mean/std hoặc median/IQR, số lần vi phạm và độ lớn vi phạm. Tách planner time,
   controller time và visualization; ghi CPU, thread, ngôn ngữ, solver và version.
8. Nếu sử dụng acceleration-level controller, qua adapter sang cùng plant lực/drag/
   gravity; không cho một phương pháp plant lý tưởng và phương pháp khác plant nhiễu.

## 5. Thứ tự triển khai được khuyến nghị

Vector field nền tảng → CBF/HOCBF → tube-constrained DMPC → AMSwarm hoặc Primitive-Swarm.
Lý do: hai bước đầu dùng được phần lớn benchmark hiện có, bước ba khảo sát ưu thế
của dự báo, bước cuối mở rộng đối chiếu với planner hiện đại có code tác giả.

Baseline vượt tốc vẫn được giữ nguyên và công khai; không chỉnh riêng nó hoặc riêng
đối thủ trên test set rồi coi đó là so sánh công bằng. Các bảo đảm lý thuyết của công
trình khác phải được đọc kèm điều kiện mô hình, truyền thông và khả thi solver trước
khi đưa ra tuyên bố an toàn tương đương.
