# opencamlib Python bindings (Rust)

PyO3 bindings for the Rust port of OpenCAMLib. They mirror the Python API of the original
C++ `opencamlib` package (its current pybind11 bindings): the same classes, method names,
argument order and enums. Existing scripts run unchanged:

```python
import opencamlib as ocl          # or: from opencamlib import ocl

surf = ocl.STLSurf()
ocl.STLReader("model.stl", surf)
cutter = ocl.BallCutter(2.0, 10.0)

wl = ocl.Waterline()
wl.setSTL(surf)
wl.setCutter(cutter)
wl.setZ(1.5)
wl.setSampling(0.1)
wl.run()
for loop in wl.getLoops():
    print(len(loop), "points")
```

## Building

The package is built with [maturin](https://www.maturin.rs). It produces one abi3 wheel
for CPython 3.9 and later.

```bash
pip install maturin
cd python
maturin develop --release   # install into the active virtualenv
maturin build --release     # or build a wheel into ../target/wheels
```

The package is named `opencamlib`, the same as the C++ package, so install it in its own
virtualenv.

## Differences from the C++ bindings

- **Results** are the same as the Rust library's, which match the C++ library except
  where C++ bugs are fixed (cone and composite cutter gouging, the STL reader hang). See the
  top-level README.
- **Speed**: `run()` releases the GIL, so operations can run in parallel from Python threads.
  Each operation is also multi-threaded internally. On the tux model, `BatchDropCutter.run()`
  on 271k points takes 0.49 s vs 2.83 s, and three waterlines take 0.38 s vs 1.17 s.
- **Errors** become Python exceptions instead of crashes or silent failures:
  - calling `run()` before `setSTL()`/`setCutter()`/`setPath()`, or a `BatchPushCutter`
    without `setXDirection()`/`setYDirection()`, raises `RuntimeError`;
  - invalid cutter dimensions raise `ValueError`;
  - `STLReader` raises `OSError` for an unreadable file;
  - `offsetCutter()` on composite cutters raises `NotImplementedError`.
- **Copies:** attributes that return points (`line.p1`, `triangle.p`, ...) return copies, so
  `line.p1.x = 5` does not change the line. Assign the whole point instead
  (`line.p1 = ocl.Point(5, 0, 0)`). Setting `p1`, `p2`, `c` or `dir` on an `Arc` updates its
  length and radius.
- **Enums:** members (`ocl.CCType.FACET`, `ocl.FACET`, ...) compare and hash equal, but are
  not singletons, so compare them with `==`, not `is`.
- **Small extras:** `ConeCutter`'s length argument defaults to 10 (as in C++), `dropCutterSTL`
  is available on every cutter, and `Point` has a `__repr__`.
- **Fibers:** `getXFibers()`/`getYFibers()` return all fibers, including empty ones, as the
  current C++ bindings do. The older Boost.Python wheel (2023.1.11 on PyPI) left out empty
  fibers.
- **Reruns:** re-running a `Waterline` at another z does not accumulate fibers from earlier
  runs.

## Tests

```bash
python -m unittest discover -s tests          # API and behaviour tests
```

`tests/dump_results.py` and `tests/compare_results.py` compare this package with the C++
wheel on a model, using drop-cutter, push-cutter, waterline and path operations:

```bash
python tests/dump_results.py model.stl rust.json     # env with these bindings
python tests/dump_results.py model.stl cpp.json      # env with `pip install opencamlib`
python tests/compare_results.py cpp.json rust.json
```

On `gnu_tux_mod.stl` all 72 sections are identical, except the drop-cutter results of
cutters that contain a cone. Those are raised (never lowered) by the gouging fixes.
