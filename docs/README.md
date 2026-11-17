# Tài liệu dự án

Bài **Curve Virtual Tube** là nền tảng chính. Các mô phỏng trong repo là thí
nghiệm phụ; hướng đang nghiên cứu là planner tube động né vùng vật cản quét qua.

Chỉ dẫn cho các phiên Codex nằm ở [AGENTS.md](../AGENTS.md), gồm phạm vi nghiên
cứu, các phát hiện quan trọng, sở thích làm việc và đường dẫn context hiện hành.

## Đọc theo mục tiêu

| Mục tiêu | Tài liệu |
|---|---|
| Bắt đầu với bài báo chính | [paper/CONTEXT.md](paper/CONTEXT.md) |
| Hiểu SDF → enhanced RRT* → minimum snap → tube | [paper/TUBE_PLANNING.md](paper/TUBE_PLANNING.md) |
| Hiểu hiện thực bài báo và cách chạy V-B | [paper/IMPLEMENTATION.md](paper/IMPLEMENTATION.md) |
| Xem sai khác công thức và các sửa đổi | [paper/FORMULA_AUDIT.md](paper/FORMULA_AUDIT.md), [paper/FORMULA_FIXES.md](paper/FORMULA_FIXES.md) |
| Làm tiếp planner tube động | [dynamic_tube/PLANNER.md](dynamic_tube/PLANNER.md) |
| Hiểu công thức slide 6–10 của báo cáo horizon | [dynamic_tube/SLIDE_FORMULAS.md](dynamic_tube/SLIDE_FORMULAS.md) |
| Theo dõi vấn đề chưa xử lý và việc để sau | [dynamic_tube/ISSUES.md](dynamic_tube/ISSUES.md) |
| Xem thí nghiệm outer DMPC | [experiments/dmpc/IMPLEMENTATION.md](experiments/dmpc/IMPLEMENTATION.md) |
| Xem sửa DMPC v2 và audit | [experiments/dmpc/FIXES.md](experiments/dmpc/FIXES.md), [experiments/dmpc/AUDIT.md](experiments/dmpc/AUDIT.md) |
| Tra cứu phương pháp liên quan | [research/RELATED_METHODS.md](research/RELATED_METHODS.md) |
| Đọc lịch sử trao đổi ban đầu | [research/CONVERSATION_CONTEXT.md](research/CONVERSATION_CONTEXT.md) |
| Hiểu cấu trúc code | [ARCHITECTURE.md](ARCHITECTURE.md) |

## Quy ước cập nhật

- `dynamic_tube/PLANNER.md` là tài liệu hiện hành duy nhất cho lý thuyết, cách
  triển khai, lệnh chạy và kết quả swept-volume planner. Không tạo thêm file plan
  mới cho mỗi lần thảo luận.
- `dynamic_tube/ISSUES.md` giữ các mã DT-xx, quyết định, việc chưa làm và nguồn
  tham khảo cho các phần chủ động để sau.
- Theo dõi hằng ngày theo hai hướng lớn A (horizon), B (không gian–thời gian).
  Bốn bước tiếp theo A1–A4 nằm trong horizon; mã DT-xx cũ giữ ở bảng tra cứu.
- Các audit/kết quả cũ giữ rõ ngày và phạm vi; không dùng chúng thay kết quả mới
  của cùng phương pháp. Nhật ký trao đổi là lịch sử, không phải status hiện hành.
- Config và artifacts chứa dữ liệu chạy; docs giải thích cách dùng và giới hạn.

## Nội dung đã gộp hoặc bỏ

- Bỏ kế hoạch DMPC ban đầu vì đã được triển khai và có implementation/fixes/audit.
- Gộp thảo luận kiến trúc tube động vào phần việc để sau trong `ISSUES.md`, giữ
  các lựa chọn kiến trúc, giới hạn và nguồn tham khảo.
- Rút gọn swept-volume planner: bỏ workflow đề xuất cũ bị lặp với implementation;
  giữ formulation, công thức, giả thiết, thực hành, kết quả và giới hạn.
