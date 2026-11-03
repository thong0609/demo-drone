# Cấu trúc dự án

```text
configs/
  scenarios/                 # Bản đồ, điểm đầu–đích, thông số tube dùng chung
  experiments/               # Cấu hình phép thử, hiện có paper_vb.json
drone_bench/
  common/vectors.py          # Phép toán vector
  environment/
    obstacles.py             # Hình học vật cản
    world.py                 # World, khoảng cách và kiểm tra va chạm
  tubes/model.py             # Dữ liệu tube, phép chiếu, kiểm tra khoảng hở
  algorithms/
    planners/                # reference, enhanced_rrt, minimum_snap, paper_tube
    controllers/             # paper_outer.py, paper_inner.py, potentials.py
  simulation/                # Dynamics, khởi tạo, vòng lặp, metrics và interfaces
  io/benchmark.py            # Đọc/ghi cấu hình và benchmark cố định
  visualization/
    mesh.py                  # Mesh chỉ phục vụ hiển thị
    viewer.html              # Template trình xem
    export.py                # Xuất HTML offline
  cli/                       # build_environment.py, run_paper_vb.py
  build.py                   # Giữ tương thích với lệnh cũ
experiments/                 # Kịch bản chạy và so sánh sẽ thêm sau
artifacts/                   # Dữ liệu và HTML sinh ra, không sửa bằng tay
tests/                       # Kiểm thử
README.md                    # Bắt đầu từ đây
docs/PAPER_CONTEXT.md        # Ghi chú bài báo
```

## Luồng hiện tại

`configs/scenarios` → `io.load_scenario` → `Environment` →
`algorithms.planners.reference` → `Tube` → kiểm tra hình học →
`artifacts/benchmark.json` + `artifacts/viewer.html`.

Dựng môi trường từ thư mục gốc:

```bash
python3 -m drone_bench.cli.build_environment
python3 -m unittest discover -s tests -v
```

Lệnh cũ `python3 -m drone_bench.build` vẫn hoạt động.

## Thêm thuật toán sau này

- Mỗi planner có module riêng trong `algorithms/planners/`; nhận môi trường
  dùng chung và trả về đối tượng `Tube` khi so sánh tube planners.
- Mỗi controller có module riêng trong `algorithms/controllers/`; dùng cùng
  world, tube và trạng thái UAV do bộ mô phỏng cung cấp.
- Dynamics và vòng lặp thời gian đặt trong `simulation/`, không lặp lại trong
  từng controller. API hiện tại nằm trong `simulation/interfaces.py`.
- Cấu hình thử nghiệm nằm trong `configs/experiments/`; kịch bản điều phối
  nằm trong `experiments/`. Kịch bản không chứa lại công thức hình học.
- Tách đầu ra mỗi lần chạy bằng `--output artifacts/<tên-lần-chạy>`.
- Khi so sánh controller, nạp cùng benchmark đã xuất bằng
  `load_benchmark`; không tự sinh tube khác cho từng thuật toán.

```python
from drone_bench.io.benchmark import load_benchmark

environment, tube = load_benchmark('artifacts/benchmark.json')
clearance = environment.clearance([50, 140, 50])
projection = tube.project([50, 140, 50])
```

Đã triển khai thí nghiệm V-B trong các module planners, controllers và simulation.
`experiments/` vẫn dành cho kịch bản so sánh nhiều thuật toán. Mô hình hình học
không phụ thuộc vào thuật toán, CLI hay mã trực quan hóa.

Chạy thí nghiệm: `python3 -m drone_bench.cli.run_paper_vb`.
Chi tiết công thức và giả định: [PAPER_VB_IMPLEMENTATION.md](PAPER_VB_IMPLEMENTATION.md).


## Outer-loop comparison

`algorithms/controllers/dmpc/` contains predictor, tube references, constraints,
coordination and a native QP backend. `cli/compare_outer.py` freezes shared inputs,
runs either controller with the unchanged inner/plant, and exports comparison.
`visualization/comparison.py` generates common-axis SVG overlays and a report.
See [DMPC_IMPLEMENTATION.md](DMPC_IMPLEMENTATION.md).
