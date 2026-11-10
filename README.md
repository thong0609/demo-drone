# Shared curve-tube UAV benchmark

Static 3D environment and reference tube, independent of planners and controllers.

See [Cấu trúc dự án và cách mở rộng](docs/ARCHITECTURE.md) for the module map and algorithm conventions.
Python 3 standard library only; no downloads or server required.

## Run

```bash
python3 -m drone_bench.build
python3 -m unittest discover -s tests -v
```

Open `artifacts/viewer.html` in a browser. Drag to rotate, scroll to zoom,
select cross-sections with the slider. The HTML embeds its data and works offline.
Canvas uses an orthographic projection and depth-sorted translucent polygons;
it is a qualitative viewer, not a collision checker.

## Fixed benchmark contract

- `configs/scenarios/benchmark_v1.json`: authoritative world geometry, endpoints, safety
  radius and reference-tube generation settings. All positions and lengths are metres.
- `artifacts/benchmark.json`: versioned algorithm-neutral export containing world,
  sampled tube, radii, and SHA-256 of canonical scenario JSON.
- `artifacts/validation.json`: geometric clearance certificate and tube statistics.
- `drone_bench/environment/world.py` and `drone_bench/tubes/model.py`: `Environment.clearance(p)`,
  `Environment.segment_free(a,b,radius,spacing)`, `Tube.project(p)`,
  `Tube.validate(env,margin)`.

For controller comparisons, load **the same exported tube** and scenario for every
algorithm; do not regenerate a route for each controller. For planner comparisons,
use the same scenario and compare the newly generated paths/tubes. Freeze the export
and record its file hash alongside results if externally replacing a tube. Initial
swarm states, noise profiles, time step, and dynamics must also be shared during simulation comparisons. The V-B configuration and initial_state export
now define these for the paper controller baseline.

## Geometry conventions

World axes: X/Y horizontal, Z up. World limits are collision boundaries.
Obstacles are axis-aligned solid boxes and solid spheres. Clearance is positive in
free space, negative within an obstacle or outside the world. It is computed
analytically, without a voxel grid or interpolation artifacts.

Reference centerline: interpolate start to goal and add deterministic sinusoidal
Y/Z offsets. This is **not enhanced RRT* or minimum snap**. Radius is capped at
45 m and shrinks with obstacle/world clearance, with a discretization reserve.
The exported object uses a piecewise-linear centerline and linearly interpolated
radius. Normal disks form its cross-sections; the viewer uses smoothly varying
normals and 24-sided rings as an approximation.

Safety certification encloses the tube in swept balls along each segment. Since
free-space clearance is 1-Lipschitz, for endpoints a,b and radii ra,rb,

```
clearance(any point on segment) - radius
  >= min(clearance(a), clearance(b)) - length(a,b)/2 - max(ra,rb)
```

The build rejects tubes violating the requested obstacle/world margin. This checks
whole segments, not just a plotted mesh. It does not certify tube injectivity,
curvature constraints, UAV dynamics, or swarm safety. `Tube.project` returns nearest
centerline projection and local radial clearance; it is not the exact global signed
distance to the tube boundary, particularly at bends/end caps. Controllers should
maintain an additional 5 m radial clearance for UAV safety balls. Endpoint disks are
open entry/exit interfaces, not physical walls.

## Relation to the paper

Dimensions (500 × 500 × 300 m), start (50,140,50), goal (460,390,170), maximum
radius (45 m) and UAV safety radius (5 m) follow Section V-B. Obstacle coordinates
and the sinusoidal reference route are a **custom reproducible benchmark**, not an
exact reconstruction of Figure 7. See `docs/PAPER_CONTEXT.md` for paper notes.

## Paper Section V-B simulation

```bash
python3 -m drone_bench.cli.run_paper_vb
```

Open `artifacts/paper_vb_continuous/report.html` for plots, snapshots and interactive replay.
The historical sampled-tube results remain in `artifacts/paper_vb/`. See
[the formula corrections](docs/PAPER_FORMULA_FIXES.md) for the continuous geometry,
bounded NN specification and explicit assumptions used by the current configuration.
The command exports results even if a constraint fails, returning exit code 2.
The baseline completes all 27 UAVs but exceeds the actual 11 m/s speed limit;
command saturation alone does not ensure actual speed compliance.

See [Implementation, equations, assumptions and results](docs/PAPER_VB_IMPLEMENTATION.md).
Planners and controllers are in separate modules; the runner accepts interchangeable
outer/inner controllers with a common state and diagnostics interface.


## Compare outer loops: vector field versus DMPC

```bash
python3 -m drone_bench.cli.compare_outer
```

Open `artifacts/outer_comparison/report.html`. Both runs load the same frozen
benchmark and complete inner initial states; only the outer controller changes.
The DMPC backend needs gcc and compiles locally; no Python numerical packages or
network downloads are needed. Run only one branch with `--only dmpc` or
`--only vector_field`. All safety violations and solver failures remain in the
reports. See [DMPC implementation](docs/DMPC_IMPLEMENTATION.md) for prediction,
coordination, solver settings, timing limitations and comparison scope.

DMPC v2 fixes and short-before-full workflow:

```bash
python3 -m drone_bench.cli.validate_dmpc
python3 -m drone_bench.cli.compare_outer --only dmpc --validation artifacts/dmpc_validation/gates.json
```

See [DMPC fixes](docs/DMPC_FIXES.md). Validation fingerprints reject stale gates;
failed outer updates save an exact causal checkpoint for one-cycle reproduction.
