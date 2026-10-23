"""Run a set of OpenCAMLib operations and dump the results as JSON.

Works with both the C++ `opencamlib` wheel and these Rust bindings, so the two can be
compared with compare_results.py:

    python dump_results.py path/to/model.stl cpp.json   # in an env with the C++ wheel
    python dump_results.py path/to/model.stl rust.json  # in an env with these bindings
    python compare_results.py cpp.json rust.json
"""
import json
import math
import sys

import opencamlib as ocl

STL, OUT = sys.argv[1], sys.argv[2]
PI = math.pi


def cutters():
    return {
        "cyl": ocl.CylCutter(1.0, 5.0),
        "ball": ocl.BallCutter(1.0, 5.0),
        "bull": ocl.BullCutter(1.0, 0.2, 5.0),
        "cone": ocl.ConeCutter(1.0, PI / 4, 5.0),
        "compcyl": ocl.CompCylCutter(1.0, 5.0),
        "compball": ocl.CompBallCutter(1.0, 5.0),
        "cylcone": ocl.CylConeCutter(0.4, 1.0, PI / 4),
        "ballcone": ocl.BallConeCutter(0.4, 1.0, PI / 4),
        "bullcone": ocl.BullConeCutter(0.4, 0.1, 1.0, PI / 4),
        "conecone": ocl.ConeConeCutter(0.4, PI / 3, 1.0, PI / 6),
    }


def xyz(p):
    return [p.x, p.y, p.z]


def cl(p):
    return [p.x, p.y, p.z, int(p.getCC().type)]


def canonical_loops(loops):
    """Loops as point lists, each rotated to start at its smallest point, then sorted."""
    out = []
    for lp in loops:
        pts = [xyz(p) for p in lp]
        if pts:
            k = min(range(len(pts)), key=lambda i: pts[i])
            pts = pts[k:] + pts[:k]
        out.append(pts)
    return sorted(out)


def main():
    s = ocl.STLSurf()
    ocl.STLReader(STL, s)
    b = s.getBounds()
    res = {"size": s.size(), "bounds": list(b)}
    cs = cutters()

    # drop-cutter on a grid
    for name, c in cs.items():
        bdc = ocl.BatchDropCutter()
        bdc.setSTL(s)
        bdc.setCutter(c)
        for i in range(31):
            for j in range(31):
                x = b[0] - 0.5 + (b[1] - b[0] + 1) * i / 30
                y = b[2] - 0.5 + (b[3] - b[2] + 1) * j / 30
                bdc.appendPoint(ocl.CLPoint(x, y, b[4] - 1))
        bdc.run()
        res[f"drop/{name}"] = [cl(p) for p in bdc.getCLPoints()]

    z = b[4] + 0.6 * (b[5] - b[4])
    for name in ("cyl", "ball", "bull", "cone"):
        c = cs[name]
        # push-cutter fibers
        for direction in "xy":
            bpc = ocl.BatchPushCutter()
            if direction == "x":
                bpc.setXDirection()
            else:
                bpc.setYDirection()
            bpc.setSTL(s)
            bpc.setCutter(c)
            for k in range(60):
                if direction == "x":
                    y = b[2] - 1 + (b[3] - b[2] + 2) * k / 59
                    bpc.appendFiber(ocl.Fiber(ocl.Point(b[0] - 1, y, z), ocl.Point(b[1] + 1, y, z)))
                else:
                    x = b[0] - 1 + (b[1] - b[0] + 2) * k / 59
                    bpc.appendFiber(ocl.Fiber(ocl.Point(x, b[2] - 1, z), ocl.Point(x, b[3] + 1, z)))
            bpc.run()
            res[f"push/{name}/{direction}"] = [
                [[i.lower, i.upper, int(i.lower_cc.type), int(i.upper_cc.type)] for i in f.getInts()]
                for f in bpc.getFibers()
            ]
            res[f"push_cl/{name}/{direction}"] = sorted(cl(p) for p in bpc.getCLPoints())
        # waterlines
        for kind in ("Waterline", "AdaptiveWaterline"):
            for run in ("run", "run2"):
                wl = getattr(ocl, kind)()
                wl.setSTL(s)
                wl.setCutter(c)
                wl.setZ(z)
                wl.setSampling(0.1)
                getattr(wl, run)()
                res[f"{kind}/{run}/{name}"] = canonical_loops(wl.getLoops())
                # count non-empty fibers: the 2023 C++ wheel leaves out empty ones
                res[f"{kind}/{run}/{name}/nfibers"] = [
                    sum(1 for f in wl.getXFibers() if f.getInts()),
                    sum(1 for f in wl.getYFibers() if f.getInts()),
                ]
        # path drop-cutters
        path = ocl.Path()
        ym = 0.5 * (b[2] + b[3])
        path.append(ocl.Line(ocl.Point(b[0] - 1, ym, b[4]), ocl.Point(b[1] + 1, ym, b[4])))
        path.append(ocl.Arc(ocl.Point(b[1] + 1, ym, b[4]), ocl.Point(b[1] + 1, ym + 2, b[4]),
                            ocl.Point(b[1] + 1, ym + 1, b[4]), True))
        for kind in ("PathDropCutter", "AdaptivePathDropCutter"):
            pdc = getattr(ocl, kind)()
            pdc.setSTL(s)
            pdc.setCutter(c)
            pdc.setPath(path)
            pdc.setZ(b[4])
            pdc.setSampling(0.1)
            if kind.startswith("Adaptive"):
                pdc.setMinSampling(0.01)
            pdc.run()
            res[f"{kind}/{name}"] = [cl(p) for p in pdc.getCLPoints()]

    # single-triangle cutter methods
    t = ocl.Triangle(ocl.Point(0, 0, 0), ocl.Point(3, 0.5, 1), ocl.Point(0.5, 3, 2))
    single = []
    for name, c in cs.items():
        for method in ("vertexDrop", "facetDrop", "edgeDrop", "dropCutter"):
            p = ocl.CLPoint(1.0, 1.0, -10)
            hit = getattr(c, method)(p, t)
            single.append([name, method, bool(hit)] + cl(p))
        f = ocl.Fiber(ocl.Point(-5, 1.0, 0.5), ocl.Point(5, 1.0, 0.5))
        i = ocl.Interval()
        hit = c.pushCutter(f, i, t)
        single.append([name, "pushCutter", bool(hit), i.lower, i.upper])
        single.append([name, "radius", c.getRadius(), c.getDiameter(), c.getLength()])
    res["single"] = single
    res["str"] = [str(c) for n, c in cs.items() if n in ("cyl", "ball", "bull")]
    res["offset"] = [str(cs[n].offsetCutter(0.1)) for n in ("cyl", "ball", "bull")]
    p = ocl.CLPoint(1.0, 1.0, -10)
    cs["ball"].dropCutterSTL(p, s)
    res["dropCutterSTL"] = cl(p)

    # geometry helpers
    a, q = ocl.Point(1, 2, 3), ocl.Point(-1, 0.5, 2)
    a.normalize()
    res["point"] = [xyz(a + q), xyz(a - q), xyz(a * 2.0), xyz(2.0 * a), a.dot(q), xyz(a.cross(q)),
                    a.norm(), a.xyNorm(), a.xyDistance(q), a.isRight(ocl.Point(0, 0, 0), q),
                    (ocl.Point(0.5, 0.5, 0).isInside(t) if hasattr(ocl.Point, "isInside") else None), str(ocl.Point(1, 0.5, 1.0 / 3))]
    res["triangle"] = [xyz(t.n)] + [xyz(v) for v in t.getPoints()]
    filt = ocl.LineCLFilter()
    for k in range(20):
        filt.addCLPoint(ocl.CLPoint(k, 0.001 * (k % 3), 0))
    filt.addCLPoint(ocl.CLPoint(20, 5, 0))
    filt.setTolerance(0.01)
    filt.run()
    res["lineclfilter"] = [xyz(p) for p in filt.getCLPoints()]
    zz = ocl.ZigZag()
    zz.setDirection(ocl.Point(1, 0, 0))
    zz.setOrigin(ocl.Point(0, 0, 0))
    zz.setStepOver(0.7)
    for v in [(0, 0), (5, 0), (5, 4), (0, 4)]:
        zz.addPoint(ocl.Point(v[0], v[1], 0))
    zz.run()
    res["zigzag"] = [xyz(p) for p in zz.getOutput()]
    iv = ocl.Interval()
    iv.updateUpper(0.7, ocl.CCPoint(1, 2, 3))
    iv.updateLower(0.2, ocl.CCPoint(4, 5, 6))
    fb = ocl.Fiber(ocl.Point(0, 0, 0), ocl.Point(10, 0, 0))
    fb.addInterval(iv)
    res["interval"] = [iv.lower, iv.upper, xyz(iv.lower_cc), xyz(fb.point(0.5)), len(fb.getInts()), str(iv)]
    ep = ocl.EllipsePosition()
    ep.setDiangle(0.7)
    e = ocl.Ellipse(ocl.Point(0, 1, 0), 3.0, 1.0, 0.5)
    res["ellipse"] = [ep.s, ep.t, xyz(e.ePoint(ep)), xyz(e.oePoint(ep)), xyz(e.normal(ep))]
    res["eps"] = [ocl.eps(), ocl.epsD(1.0), ocl.epsF(1.0)]

    with open(OUT, "w") as fh:
        json.dump(res, fh)


main()
