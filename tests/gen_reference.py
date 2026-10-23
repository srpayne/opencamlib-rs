"""Generate reference data for the Rust port from the C++ OpenCAMLib Python bindings.

Usage:
    pip install opencamlib==2023.1.11
    python tests/gen_reference.py <path-to-opencamlib-repo>/stl tests/data

Writes binary copies of the test models plus text files with drop-cutter, push-cutter,
waterline and path drop-cutter results, which tests/reference.rs compares against.
"""
import math
import struct
import sys

import opencamlib as ocl

SRC, OUT = sys.argv[1], sys.argv[2]

# name -> (python constructor, spec string parsed by tests/reference.rs)
PI = math.pi
CUTTERS = {
    "cyl": (lambda: ocl.CylCutter(1.0, 5.0), "cyl 1.0 5.0"),
    "ball": (lambda: ocl.BallCutter(1.0, 5.0), "ball 1.0 5.0"),
    "bull": (lambda: ocl.BullCutter(1.0, 0.2, 5.0), "bull 1.0 0.2 5.0"),
    "cone": (lambda: ocl.ConeCutter(1.0, PI / 4, 5.0), f"cone 1.0 {PI / 4!r} 5.0"),
    "compcyl": (lambda: ocl.CompCylCutter(1.0, 5.0), "compcyl 1.0 5.0"),
    "compball": (lambda: ocl.CompBallCutter(1.0, 5.0), "compball 1.0 5.0"),
    "cylcone": (lambda: ocl.CylConeCutter(0.4, 1.0, PI / 4), f"cylcone 0.4 1.0 {PI / 4!r}"),
    "ballcone": (lambda: ocl.BallConeCutter(0.4, 1.0, PI / 4), f"ballcone 0.4 1.0 {PI / 4!r}"),
    "bullcone": (lambda: ocl.BullConeCutter(0.4, 0.1, 1.0, PI / 4), f"bullcone 0.4 0.1 1.0 {PI / 4!r}"),
    "conecone": (lambda: ocl.ConeConeCutter(0.4, PI / 3, 1.0, PI / 6), f"conecone 0.4 {PI / 3!r} 1.0 {PI / 6!r}"),
}


def load(name):
    s = ocl.STLSurf()
    ocl.STLReader(f"{SRC}/{name}", s)
    return s


def write_binary_stl(s, path):
    tris = s.getTriangles()
    with open(path, "wb") as f:
        f.write(b"binary copy of an opencamlib test model".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        for t in tris:
            f.write(struct.pack("<3f", 0, 0, 0))
            for p in t.getPoints():
                f.write(struct.pack("<3f", p.x, p.y, p.z))
            f.write(b"\0\0")


def frange(a, b, step):
    n = int(round((b - a) / step))
    return [a + i * step for i in range(n + 1)]


def dropcutter(model, s, cname, step):
    ctor, spec = CUTTERS[cname]
    c = ctor()
    b = s.getBounds()
    bdc = ocl.BatchDropCutter()
    bdc.setSTL(s)
    bdc.setCutter(c)
    z0 = b[4] - 1.0
    for x in frange(b[0] - 0.5, b[1] + 0.5, step):
        for y in frange(b[2] - 0.5, b[3] + 0.5, step):
            bdc.appendPoint(ocl.CLPoint(x, y, z0))
    bdc.run()
    with open(f"{OUT}/drop_{model}_{cname}.txt", "w") as f:
        f.write(f"# {spec}\n")
        for p in bdc.getCLPoints():
            f.write(f"{p.x!r} {p.y!r} {z0!r} {p.z!r} {p.getCC().type.real}\n")


def fibers(model, s, cname, z, sampling):
    ctor, spec = CUTTERS[cname]
    c = ctor()
    b = s.getBounds()
    r2 = 2 * c.getRadius()
    with open(f"{OUT}/push_{model}_{cname}.txt", "w") as f:
        f.write(f"# {spec}\n")
        for direction in "xy":
            bpc = ocl.BatchPushCutter()
            if direction == "x":
                bpc.setXDirection()
            else:
                bpc.setYDirection()
            bpc.setSTL(s)
            bpc.setCutter(c)
            if direction == "x":
                for y in frange(b[2] - r2, b[3] + r2, sampling):
                    bpc.appendFiber(ocl.Fiber(ocl.Point(b[0] - r2, y, z), ocl.Point(b[1] + r2, y, z)))
            else:
                for x in frange(b[0] - r2, b[1] + r2, sampling):
                    bpc.appendFiber(ocl.Fiber(ocl.Point(x, b[2] - r2, z), ocl.Point(x, b[3] + r2, z)))
            bpc.run()
            for fb in bpc.getFibers():
                ints = fb.getInts()
                f.write(f"F {direction} {fb.p1.x!r} {fb.p1.y!r} {fb.p2.x!r} {fb.p2.y!r} {z!r} {len(ints)}\n")
                for i in ints:
                    f.write(f"I {i.lower!r} {i.upper!r} {i.lower_cc.type.real} {i.upper_cc.type.real}\n")


def write_loops(f, loops):
    f.write(f"N {len(loops)}\n")
    for lp in loops:
        f.write(f"L {len(lp)}\n")
        for p in lp:
            f.write(f"{p.x!r} {p.y!r} {p.z!r}\n")


def waterline(model, s, cname, z, sampling):
    ctor, spec = CUTTERS[cname]
    c = ctor()
    with open(f"{OUT}/waterline_{model}_{cname}.txt", "w") as f:
        f.write(f"# {spec}\n")
        f.write(f"Z {z!r} {sampling!r}\n")
        for kind in ("simple", "smart", "adaptive_simple", "adaptive_smart"):
            wl = ocl.AdaptiveWaterline() if kind.startswith("adaptive") else ocl.Waterline()
            wl.setSTL(s)
            wl.setCutter(c)
            wl.setZ(z)
            wl.setSampling(sampling)
            if kind.endswith("smart"):
                wl.run2()
            else:
                wl.run()
            f.write(f"K {kind}\n")
            write_loops(f, wl.getLoops())


def path_drop(model, s, cname, z):
    ctor, spec = CUTTERS[cname]
    c = ctor()
    b = s.getBounds()
    path = ocl.Path()
    y0, y1 = b[2] + 0.3 * (b[3] - b[2]), b[2] + 0.6 * (b[3] - b[2])
    path.append(ocl.Line(ocl.Point(b[0] - 0.5, y0, z), ocl.Point(b[1] + 0.5, y0, z)))
    path.append(ocl.Line(ocl.Point(b[1] + 0.5, y0, z), ocl.Point(b[1] + 0.5, y1, z)))
    path.append(ocl.Line(ocl.Point(b[1] + 0.5, y1, z), ocl.Point(b[0] - 0.5, y1, z)))
    with open(f"{OUT}/path_{model}_{cname}.txt", "w") as f:
        f.write(f"# {spec}\n")
        f.write(f"Z {z!r}\n")
        for kind in ("uniform", "adaptive"):
            pdc = ocl.AdaptivePathDropCutter() if kind == "adaptive" else ocl.PathDropCutter()
            pdc.setSTL(s)
            pdc.setCutter(c)
            pdc.setPath(path)
            pdc.setZ(z)
            pdc.setSampling(0.1)
            if kind == "adaptive":
                pdc.setMinSampling(0.01)
            pdc.run()
            pts = pdc.getCLPoints()
            f.write(f"K {kind} {len(pts)}\n")
            for p in pts:
                f.write(f"{p.x!r} {p.y!r} {p.z!r}\n")
        f.write("P\n")
        for line in [((b[0] - 0.5, y0), (b[1] + 0.5, y0)), ((b[1] + 0.5, y0), (b[1] + 0.5, y1)), ((b[1] + 0.5, y1), (b[0] - 0.5, y1))]:
            f.write(f"{line[0][0]!r} {line[0][1]!r} {line[1][0]!r} {line[1][1]!r}\n")


if __name__ == "__main__":
    tux = load("gnu_tux_mod.stl")
    write_binary_stl(tux, f"{OUT}/tux.stl")
    demo = load("demo.stl")
    write_binary_stl(demo, f"{OUT}/demo.stl")

    for cname in CUTTERS:
        dropcutter("tux", tux, cname, 0.25)
        dropcutter("demo", demo, cname, 0.25)
    for cname in ("cyl", "ball", "bull", "cone", "cylcone", "ballcone"):
        fibers("tux", tux, cname, 1.75145, 0.1)
    for cname in ("cyl", "ball", "bull", "cone"):
        waterline("tux", tux, cname, 1.75145, 0.1)
        waterline("demo", demo, cname, 0.5, 0.1)
    for cname in ("cyl", "ball", "bull"):
        path_drop("tux", tux, cname, -3.0)
