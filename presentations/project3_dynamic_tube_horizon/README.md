# Báo cáo Project III — Tube động và horizon

Bản cập nhật nội dung từ file báo cáo trước của người dùng:
`/home/tts/Downloads/Project-III-bao-cao-tien-do-outer-loop-4-trang.pptx`.
File nguồn và bản DMPC–ADMM–DCBF trước đó được giữ nguyên.

- [PowerPoint](Project-III-Tube-dong-Horizon-bao-cao.pptx): 14 slide, giữ nhận diện
  HUST, GVHD TS. Tưởng Văn Đạt và sinh viên Tô Huy Thông 20235435.
- [PDF](Project-III-Tube-dong-Horizon-bao-cao.pdf): xuất từ chính PowerPoint bằng
  LibreOffice, không phải PDF ghép từ ảnh preview.
- [Giải thích công thức slide 6–10](../../docs/dynamic_tube/SLIDE_FORMULAS.md):
  từng ký hiệu, cách tính, ví dụ số và giới hạn.
- `contact-sheet.png`, `rendered/`: ảnh kiểm tra toàn bộ slide từ PDF kết xuất.
- `content.json`: nội dung, công thức và ghi chú thuyết trình của 12 slide chính.
- `validation.json`: kiểm tra ZIP/XML, liên kết nội bộ, số trang và provenance.
- `build_deck.py`: dựng lại PowerPoint bằng Python standard library và Pillow.

## Mạch trình bày

1. Câu hỏi vật cản động và ý tưởng thêm vector tránh vào outer loop.
2. Nguy cơ né khác phía/tách đàn và lý do chuyển sang điều chỉnh tube chung.
3. Hướng mở của tác giả về tube realtime trong môi trường động.
4. Phương pháp đã chọn: dự báo → vùng quét → enhanced RRT* → minimum snap → kiểm chứng.
5. Công thức chuyển động, vùng quét capsule, khoảng cách, kiểm tra cả thể tích tube.
6. Mixed cost/Visual và mục tiêu minimum snap, phân biệt bài gốc với adaptation.
7. Thử nhiều hướng vật cản và thay đổi horizon; số liệu lấy trực tiếp từ artifacts.
8. Giả thiết, hạn chế độ trễ/tube lúc dùng và phạm vi kết luận.
9. Mục tiêu tuần sau: thử lập kế hoạch không gian–thời gian, chưa triển khai.

Văn bản và các ô bảng có thể sửa trong PowerPoint. Công thức là ảnh PNG rõ nét;
biểu thức gốc và giải thích được giữ trong notes và `content.json`. Mỗi slide có
chú thích vai trò biến/tham số. Không gán bảo đảm an toàn hệ thống cho chứng nhận
hình học hoặc coi việc tách đàn là kết quả đã được thí nghiệm chứng minh.

## Nguồn

- Yuan et al., *Curve Virtual Tube Planning and Distributed Control for Swarm UAVs
  With Complex Constraints and Disturbances*, IEEE TIE 2026,
  DOI 10.1109/TIE.2025.3603043; đặc biệt Section III và Section VI.
- [Lý thuyết và thí nghiệm hiện hành](../../docs/dynamic_tube/PLANNER.md).
- `artifacts/swept_volume_direction_evaluation/summary.json`: 17 ca/612 lượt.
- `artifacts/swept_horizon_study/summary.json`: 15 ca, 6 H, 540 lượt.

## Dựng lại

Từ gốc repo:

```bash
python3 presentations/project3_dynamic_tube_horizon/build_deck.py
libreoffice --headless --convert-to pdf --outdir /tmp \
  presentations/project3_dynamic_tube_horizon/Project-III-Tube-dong-Horizon-bao-cao.pptx
```

Build cần các artifacts đầu vào đã tồn tại; không tự chạy lại nghiên cứu để tạo
số liệu mới. Khi dựng lại, cập nhật PDF kết xuất và validation cho cùng bản PPTX.
Hiện đã kiểm tra không có cảnh báo tràn khung từ trình dựng, xuất thành công và
xem đủ 14 trang LibreOffice; chưa kiểm tra bằng Microsoft PowerPoint.
