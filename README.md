# opencamlib (Rust)

A Rust port of [OpenCAMLib](https://github.com/aewallin/opencamlib), a library for creating
3D toolpaths for CNC machines.

- **Drop-cutter**: drop a cutter at an (x, y) position until it touches the model
  (`PointDropCutter`, `BatchDropCutter`, `PathDropCutter`, `AdaptivePathDropCutter`).
- **Push-cutter**: push a cutter along X/Y fibers at constant z (`FiberPushCutter`,
  `BatchPushCutter`).
- **Waterline**: constant-z contour loops built from push-cutter fibers with a weave graph
  (`Waterline`, `AdaptiveWaterline`, `Weave`).
- **Cutters**: `CylCutter`, `BallCutter`, `BullCutter`, `ConeCutter` and `CompositeCutter`
  (`cyl_cone`, `ball_cone`, `bull_cone`, `cone_cone`, `comp_cyl`, `comp_ball`).
- Also ported: STL reading (ASCII and binary), kd-tree, `LineCLFilter`, `ZigZag`,
  `TSPSolver`, `CutterLocationSurface`.

The only dependency is [rayon](https://crates.io/crates/rayon), which replaces OpenMP. It is
behind the default `parallel` feature; build with `--no-default-features` to run
single-threaded.

## Usage

```rust
use opencamlib::{BallCutter, BatchDropCutter, CLPoint, STLSurf, Waterline};

let surf = STLSurf::from_file("model.stl")?;
let cutter = BallCutter::new(2.0, 10.0);

// drop-cutter
let mut bdc = BatchDropCutter::new(&surf, &cutter);
bdc.append_point(CLPoint::new(1.0, 2.0, -10.0)); // z is the lowest allowed position
bdc.run();
let z = bdc.get_cl_points()[0].z;

// waterline
let mut wl = Waterline::new(&surf, &cutter);
wl.set_z(1.5);
wl.set_sampling(0.1);
wl.run();
for lp in wl.get_loops() {
    println!("loop with {} points", lp.len());
}
```

Runnable examples:

```bash
cargo run --release --example dropcutter -- tests/data/tux.stl 1.0 0.05 > points.xyz
cargo run --release --example waterline -- tests/data/tux.stl 1.0 0.05 5 > loops.txt
```

## Python

`python/` contains PyO3 bindings that mirror the original `opencamlib` Python API, so
existing scripts (`import opencamlib as ocl`) run unchanged. They're built with maturin; see
[python/README.md](python/README.md).

## Differences from the C++ library

### API

- Method names are snake_case (`setSampling` → `set_sampling`, `getCLPoints` → `get_cl_points`).
- Operations take the surface and cutter in their constructor (`Waterline::new(&surf, &cutter)`)
  and borrow them, instead of `setSTL`/`setCutter` with raw pointers. The setters still exist
  for swapping them later.
- Push-cutters take a `FiberDirection::X`/`Y` instead of `setXDirection()`/`setYDirection()`.
- `MillingCutter` is a trait. The generic drop/push algorithms are default methods, and each
  cutter overrides what it needs, mirroring the C++ virtual methods.
- `CCPoint` and `CLPoint` dereference to `Point`, so `cl.x` and `cl.xy_distance(..)` work as in
  C++. A `CLPoint` holds its `CCPoint` by value, so there are no atomics and no heap allocations.
- The kd-tree stores triangle indices instead of copying triangles.
- `Weave` merges `SimpleWeave` and `SmartWeave` into one type: `build()` is the simple weave,
  `build_smart()` the smart one. `Waterline::run()`/`run2()` use them as in C++.
- Not ported: the Node.js and emscripten bindings, and the unused alternative drop-cutter
  loops (`dropCutter1`..`dropCutter4`). Python bindings are in `python/`.

### Behaviour

The port reproduces the C++ results exactly for cylindrical, ball and bull-nose cutters,
including the cutter-contact types (see Testing). These C++ defects are fixed:

1. **Cone cutters gouge sloping edges.** The edge-contact formula uses the cutter's total
   length where it should use the cone height.
2. **Cone cutters gouge steep facets.** The rim contact is offset by the total length, and it is
   never tested once the tip touches the facet. For example, a 90° V-bit dropped on a 63° facet
   is placed 5 units too low.
3. **Composite cutters can lift by exactly 1.0.** Within 1e-6 of a sub-cutter boundary,
   `CylCutter::height` is evaluated outside its radius and returns -1.
4. **Composite cutters can gouge at sub-cutter boundaries.** A sub-cutter's best contact is
   discarded when it lies outside that sub-cutter's radial band. The port also tests the
   boundary circles, which makes the composite drop exact. It also always runs the edge and
   vertex tests, since one sub-cutter's facet contact does not rule out another's edge contact.
5. **Binary STL files whose header starts with `solid` hang the C++ reader** (for example
   `stl/ktoolcav.stl`). The port detects binary files by their size.
6. Re-running a `Waterline` or `PathDropCutter` (for example at another z) no longer
   accumulates the fibers or points of earlier runs.
7. `CutterLocationSurface` sets its four corner vertices correctly.

With these fixes, results for cutters containing a cone are higher than the C++ results at
some points and never lower (apart from defect 3). A brute-force check finds no gouging.

Other notes:

- In C++, the order of waterline loops and their starting points depends on heap addresses.
  The port orders them by vertex creation, so the order is deterministic. The loops
  themselves are identical.
- Kept from C++: `AdaptivePathDropCutter` starts each point at the path's z (`set_z` is
  stored but unused), and `AdaptiveWaterline` adds no fiber for a non-flat segment that has
  reached `min_sampling`.

## Testing

```bash
cargo test
```

- **Unit tests** for the geometry, numerics, kd-tree, weave helpers and STL reader.
- **`tests/reference.rs`** compares against output from the C++ library (opencamlib 2023.01.11)
  on two models with ten cutter types: drop-cutter z-values and contact types, push-cutter
  intervals, waterline and adaptive-waterline loops (both weaves), and uniform and adaptive
  path drop-cutter points. See `tests/data/README.md` for how to regenerate the data.
- **`tests/gouge.rs`** samples the mesh under the cutter densely. For every cutter it checks
  that the Rust result neither gouges the surface nor floats above it, and it reports how much
  the C++ results gouge.

## Performance

On the tux model (22,626 triangles) with a 1 mm ball cutter on a 10-core Mac, compared with
the C++ library through its Python bindings:

| task | C++ | Rust |
| --- | --- | --- |
| drop-cutter, 271,051 points | 2.66 s | 0.47 s |
| waterline, sampling 0.02, 3 levels | 0.96 s | 0.36 s |

## License

LGPL-2.1-or-later, the same as OpenCAMLib, of which this is a derivative work.
