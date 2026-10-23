"""Tests for the Python bindings. Run with: python -m unittest discover python/tests"""
import math
import os
import tempfile
import threading
import unittest

import opencamlib as ocl
from opencamlib import ocl as ocl_module

# Classes and members of the C++ bindings, from their ocl.pyi type stub.
UPSTREAM_API = {
    'AdaptivePathDropCutter': ['getCLPoints', 'getSampling', 'getZ', 'minimumZ', 'run', 'setCosLimit', 'setCutter', 'setMinSampling', 'setPath', 'setSTL', 'setSampling', 'setZ'],
    'AdaptiveWaterline': ['getLoops', 'getThreads', 'getXFibers', 'getYFibers', 'reset', 'run', 'run2', 'setCutter', 'setMinSampling', 'setSTL', 'setSampling', 'setThreads', 'setZ'],
    'Arc': ['c', 'dir', 'p1', 'p2'],
    'BallConeCutter': [],
    'BallCutter': ['dropCutterSTL'],
    'BatchDropCutter': ['appendPoint', 'getBucketSize', 'getCLPoints', 'getCalls', 'getThreads', 'getTrianglesUnderCutter', 'run', 'setBucketSize', 'setCutter', 'setSTL', 'setThreads'],
    'BatchPushCutter': ['appendFiber', 'getBucketSize', 'getCLPoints', 'getCalls', 'getFibers', 'getOverlapTriangles', 'getThreads', 'run', 'setBucketSize', 'setCutter', 'setSTL', 'setThreads', 'setXDirection', 'setYDirection'],
    'Bbox': ['isInside', 'maxpt', 'minpt'],
    'BullConeCutter': [],
    'BullCutter': [],
    'CCPoint': ['type'],
    'CCType': ['name', 'value'],
    'CLPoint': ['cc', 'getCC'],
    'CompBallCutter': [],
    'CompCylCutter': [],
    'ConeConeCutter': [],
    'ConeCutter': [],
    'CutterLocationSurface': ['getEdges', 'getVertices', 'run', 'setCutter', 'setMinSampling', 'setSTL', 'setSampling'],
    'CylConeCutter': [],
    'CylCutter': ['dropCutterSTL'],
    'Ellipse': ['ePoint', 'normal', 'oePoint'],
    'EllipsePosition': ['s', 'setDiangle', 't'],
    'Fiber': ['addInterval', 'dir', 'getInts', 'p1', 'p2', 'point', 'printInts'],
    'Interval': ['empty', 'lower', 'lower_cc', 'updateLower', 'updateUpper', 'upper', 'upper_cc'],
    'Line': ['p1', 'p2'],
    'LineCLFilter': ['addCLPoint', 'getCLPoints', 'run', 'setTolerance'],
    'MillingCutter': ['dropCutter', 'edgeDrop', 'facetDrop', 'getDiameter', 'getLength', 'getRadius', 'offsetCutter', 'pushCutter', 'vertexDrop'],
    'Path': ['append', 'getSpans', 'getTypeSpanPairs'],
    'PathDropCutter': ['getCLPoints', 'getZ', 'minimumZ', 'run', 'setCutter', 'setPath', 'setSTL', 'setSampling', 'setZ'],
    'Point': ['cross', 'dot', 'isInside', 'isRight', 'norm', 'normalize', 'x', 'xRotate', 'xyDistance', 'xyNorm', 'y', 'yRotate', 'z', 'zRotate'],
    'STLReader': [],
    'STLSurf': ['addTriangle', 'bb', 'getBounds', 'getTriangles', 'rotate', 'size', 'tris'],
    'SimpleWeave': [],
    'SmartWeave': [],
    'SpanType': ['name', 'value'],
    'TSPSolver': ['addPoint', 'getLength', 'getOutput', 'reset', 'run'],
    'Triangle': ['getPoints', 'n', 'p'],
    'Waterline': ['getLoops', 'getThreads', 'getXFibers', 'getYFibers', 'reset', 'run', 'run2', 'setCutter', 'setSTL', 'setSampling', 'setThreads', 'setZ'],
    'Weave': ['addFiber', 'build', 'face_traverse', 'getCLVertices', 'getEdges', 'getINTVertices', 'getLoops', 'getVertices', 'getVerticesByType', 'numVertices', 'printGraph'],
    'WeaveVertexType': ['name', 'value'],
    'ZigZag': ['addPoint', 'getOutput', 'run', 'setDirection', 'setOrigin', 'setStepOver'],
}
UPSTREAM_FUNCTIONS = ['eps', 'epsD', 'epsF', 'max_threads', 'version']
UPSTREAM_CONSTANTS = ['ArcSpanType', 'LineSpanType'] + [
    'NONE', 'VERTEX', 'VERTEX_CYL', 'EDGE', 'EDGE_SHAFT', 'EDGE_HORIZ', 'EDGE_CYL', 'EDGE_BALL',
    'EDGE_CONE', 'EDGE_CONE_BASE', 'EDGE_HORIZ_CYL', 'EDGE_HORIZ_TOR', 'EDGE_POS', 'EDGE_NEG',
    'FACET', 'FACET_TIP', 'FACET_CYL', 'ERROR']


def P(x, y, z=0.0):
    return ocl.Point(x, y, z)


def pyramid():
    """Square pyramid: base 0..10 at z=0, apex (5, 5, 5)."""
    s = ocl.STLSurf()
    a, b, c, d, top = P(0, 0), P(10, 0), P(10, 10), P(0, 10), P(5, 5, 5)
    for t in [(a, b, top), (b, c, top), (c, d, top), (d, a, top), (a, c, b), (a, d, c)]:
        s.addTriangle(ocl.Triangle(*t))
    return s


class ApiSurface(unittest.TestCase):
    def test_all_upstream_names_exist(self):
        for name in list(UPSTREAM_API) + UPSTREAM_FUNCTIONS + UPSTREAM_CONSTANTS:
            self.assertTrue(hasattr(ocl, name), name)
            self.assertTrue(hasattr(ocl_module, name), name)

    def test_all_upstream_members_exist(self):
        for cls, members in UPSTREAM_API.items():
            for m in members:
                self.assertTrue(hasattr(getattr(ocl, cls), m), f"{cls}.{m}")

    def test_class_hierarchy(self):
        self.assertTrue(issubclass(ocl.CLPoint, ocl.Point))
        self.assertTrue(issubclass(ocl.CCPoint, ocl.Point))
        for name in ['CylCutter', 'BallCutter', 'BullCutter', 'ConeCutter', 'CompCylCutter',
                     'CompBallCutter', 'CylConeCutter', 'BallConeCutter', 'BullConeCutter',
                     'ConeConeCutter']:
            self.assertTrue(issubclass(getattr(ocl, name), ocl.MillingCutter), name)
        self.assertTrue(issubclass(ocl.SimpleWeave, ocl.Weave))
        self.assertTrue(issubclass(ocl.SmartWeave, ocl.Weave))


class Geometry(unittest.TestCase):
    def test_point(self):
        p = ocl.Point(1, 2, 3)
        q = ocl.Point(1, 1)
        self.assertEqual(str(p), "(1, 2, 3)")
        self.assertEqual(str(ocl.Point(1.0 / 3, 0, 0)), "(0.333333, 0, 0)")
        self.assertEqual(str(p + q), "(2, 3, 3)")
        self.assertEqual(str(2 * p), "(2, 4, 6)")
        self.assertEqual(str(p * 2), "(2, 4, 6)")
        self.assertEqual(p.dot(q), 3.0)
        self.assertEqual(str(ocl.Point(p)), "(1, 2, 3)")
        p += q
        self.assertEqual((p.x, p.y, p.z), (2, 3, 3))
        p.x = 7
        self.assertEqual(p.x, 7)
        self.assertAlmostEqual(ocl.Point(3, 4, 0).norm(), 5)
        self.assertTrue(ocl.Point(0.5, 0, 0).isInside(P(0, 0), P(1, 0)))
        t = ocl.Triangle(P(0, 0), P(1, 0), P(0, 1))
        self.assertTrue(ocl.Point(0.2, 0.2, 0).isInside(t))

    def test_cc_and_cl_points(self):
        cc = ocl.CCPoint(1, 2, 3)
        self.assertEqual(cc.type, ocl.NONE)
        cc.type = ocl.CCType.FACET
        cl = ocl.CLPoint(1, 2, 3, cc)
        self.assertEqual(cl.getCC().type, ocl.FACET)
        self.assertEqual(str(cl), "CL(1, 2, 3) cc=CC(1, 2, 3, t=14)")
        self.assertEqual(ocl.CLPoint(cl).cc().type, ocl.FACET)
        self.assertEqual(cl.xyDistance(ocl.Point(1, 2, 0)), 0.0)

    def test_enums(self):
        self.assertEqual(ocl.CCType.EDGE, ocl.EDGE)
        self.assertEqual(int(ocl.EDGE), 3)
        self.assertEqual(ocl.FACET_CYL.name, "FACET_CYL")
        self.assertEqual(ocl.FACET_CYL.value, 16)
        self.assertEqual(len({ocl.EDGE, ocl.CCType.EDGE, ocl.FACET}), 2)
        self.assertEqual(int(ocl.WeaveVertexType.FULLINT), 5)
        self.assertEqual(int(ocl.ArcSpanType), 1)

    def test_stlsurf_and_reader(self):
        s = pyramid()
        self.assertEqual(s.size(), 6)
        self.assertEqual(s.getBounds(), [0, 10, 0, 10, 0, 5])
        self.assertEqual(str(s), "STLSurf(N=6)")
        self.assertEqual(len(s.getTriangles()), 6)
        self.assertEqual(str(s.bb.maxpt), "(10, 10, 5)")
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "t.stl")
            with open(path, "w") as f:
                f.write("solid t\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\n"
                        "vertex 0 1 0\nendloop\nendfacet\nendsolid t\n")
            s2 = ocl.STLSurf()
            ocl.STLReader(path, s2)
            self.assertEqual(s2.size(), 1)
            with self.assertRaises(OSError):
                ocl.STLReader(os.path.join(d, "missing.stl"), s2)

    def test_path_spans(self):
        path = ocl.Path()
        path.append(ocl.Line(P(0, 0), P(1, 0)))
        path.append(ocl.Arc(P(1, 0), P(0, 1), P(0, 0), True))
        spans = path.getSpans()
        self.assertIsInstance(spans[0], ocl.Line)
        self.assertIsInstance(spans[1], ocl.Arc)
        pairs = path.getTypeSpanPairs()
        self.assertEqual([t for t, _ in pairs], [ocl.LineSpanType, ocl.ArcSpanType])
        self.assertEqual(len(ocl.Path(path).getSpans()), 2)

    def test_interval_and_fiber(self):
        i = ocl.Interval()
        self.assertTrue(i.empty())
        a, b = ocl.CCPoint(1, 2, 3), ocl.CCPoint(4, 5, 6)
        a.type = b.type = ocl.VERTEX  # an untyped (NONE) cc-point resets the interval
        i.updateUpper(0.7, a)
        i.updateLower(0.2, b)
        self.assertEqual((i.lower, i.upper), (0.2, 0.7))
        f = ocl.Fiber(P(0, 0), P(10, 0))
        f.addInterval(i)
        self.assertEqual(len(f.getInts()), 1)
        self.assertEqual(str(f.point(0.5)), "(5, 0, 0)")


class Cutters(unittest.TestCase):
    def test_drop_in_place(self):
        t = ocl.Triangle(P(0, 0), P(10, 0), P(0, 10))
        c = ocl.CylCutter(2.0, 5.0)
        cl = ocl.CLPoint(2, 2, -5)
        self.assertTrue(c.dropCutter(cl, t))
        self.assertEqual(cl.z, 0.0)
        self.assertEqual(cl.getCC().type, ocl.FACET)

    def test_str_and_dimensions(self):
        self.assertEqual(str(ocl.BallCutter(2.545, 5)), "BallCutter(d=2.545, r=1.2725, L=5)")
        c = ocl.BullCutter(4, 1, 20)
        self.assertEqual((c.getDiameter(), c.getRadius(), c.getLength()), (4, 2, 20))
        self.assertEqual(str(ocl.CylCutter(2, 5).offsetCutter(0.5)), "BullCutter(d=3, r1=1 r2=0.5, L=5.5)")
        self.assertEqual(ocl.ConeCutter(2, math.pi / 4).getLength(), ocl.ConeCutter(2, math.pi / 4, 10).getLength())
        with self.assertRaises(NotImplementedError):
            ocl.CylConeCutter(1, 2, 0.5).offsetCutter(0.1)

    def test_invalid_arguments(self):
        with self.assertRaises(ValueError):
            ocl.CylCutter(-1, 5)
        with self.assertRaises(ValueError):
            ocl.BullCutter(2, 1, 5)  # corner radius must be < d/2
        with self.assertRaises(ValueError):
            ocl.ConeCutter(2, 2.0, 5)
        with self.assertRaises(TypeError):
            ocl.MillingCutter()


class Operations(unittest.TestCase):
    def test_batch_drop_cutter_matches_brute_force(self):
        s = pyramid()
        for c in [ocl.CylCutter(1, 5), ocl.BallCutter(1, 5), ocl.BullCutter(1, 0.2, 5),
                  ocl.ConeCutter(1, math.pi / 4), ocl.BallConeCutter(0.4, 1, math.pi / 4)]:
            bdc = ocl.BatchDropCutter()
            bdc.setSTL(s)
            bdc.setCutter(c)
            pts = [(x * 0.7, y * 0.7) for x in range(-2, 17) for y in range(-2, 17)]
            for x, y in pts:
                bdc.appendPoint(ocl.CLPoint(x, y, -1))
            bdc.run()
            out = bdc.getCLPoints()
            self.assertEqual(len(out), len(pts))
            for p in out:
                ref = ocl.CLPoint(p.x, p.y, -1)
                c.dropCutterSTL(ref, s)
                self.assertAlmostEqual(p.z, ref.z, places=12, msg=str(c))
            self.assertGreater(bdc.getCalls(), 0)

    def test_set_stl_keeps_a_reference(self):
        s = ocl.STLSurf()
        bdc = ocl.BatchDropCutter()
        bdc.setSTL(s)
        bdc.setCutter(ocl.CylCutter(1, 5))
        bdc.appendPoint(ocl.CLPoint(1, 1, -1))
        s.addTriangle(ocl.Triangle(P(0, 0, 2), P(5, 0, 2), P(0, 5, 2)))  # added after setSTL
        bdc.run()
        self.assertEqual(bdc.getCLPoints()[0].z, 2.0)

    def test_run_before_setup_raises(self):
        with self.assertRaises(RuntimeError):
            ocl.BatchDropCutter().run()
        bpc = ocl.BatchPushCutter()
        bpc.setSTL(pyramid())
        bpc.setCutter(ocl.CylCutter(1, 5))
        with self.assertRaises(RuntimeError):
            bpc.run()  # no direction set

    def waterline(self, cls=ocl.Waterline, run="run"):
        wl = cls()
        wl.setSTL(pyramid())
        wl.setCutter(ocl.CylCutter(1, 5))
        wl.setZ(2.5)
        wl.setSampling(0.1)
        getattr(wl, run)()
        return wl.getLoops()

    def test_waterline(self):
        for cls in (ocl.Waterline, ocl.AdaptiveWaterline):
            for run in ("run", "run2"):
                loops = self.waterline(cls, run)
                self.assertEqual(len(loops), 1, f"{cls.__name__}.{run}")
                lp = loops[0]
                self.assertTrue(all(p.z == 2.5 for p in lp))
                # the section at z=2.5 is the square 2.5..7.5, offset by the 0.5 radius
                self.assertAlmostEqual(min(p.x for p in lp), 2.0, places=6)
                self.assertAlmostEqual(max(p.y for p in lp), 8.0, places=6)

    def test_weave_from_push_cutter_fibers(self):
        s, c, z = pyramid(), ocl.CylCutter(1, 5), 2.5
        weave = ocl.SimpleWeave()
        for direction in "xy":
            bpc = ocl.BatchPushCutter()
            getattr(bpc, f"set{direction.upper()}Direction")()
            bpc.setSTL(s)
            bpc.setCutter(c)
            for k in range(141):
                v = -2 + k * 0.1
                if direction == "x":
                    bpc.appendFiber(ocl.Fiber(P(-2, v, z), P(12, v, z)))
                else:
                    bpc.appendFiber(ocl.Fiber(P(v, -2, z), P(v, 12, z)))
            bpc.run()
            for f in bpc.getFibers():
                weave.addFiber(f)
        weave.build()
        weave.face_traverse()
        self.assertEqual(len(weave.getLoops()), 1)
        self.assertGreater(weave.numVertices(), 0)
        self.assertEqual(len(weave.getCLVertices()), len(weave.getVerticesByType(ocl.WeaveVertexType.CL)))

    def test_runs_in_parallel_threads(self):
        """run() releases the GIL; concurrent runs give the same result."""
        expected = self.waterline()
        results = [None] * 4

        def work(k):
            results[k] = self.waterline()

        threads = [threading.Thread(target=work, args=(k,)) for k in range(4)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        for r in results:
            self.assertEqual([[(p.x, p.y) for p in lp] for lp in r],
                             [[(p.x, p.y) for p in lp] for lp in expected])

    def test_path_drop_cutters(self):
        s, c = pyramid(), ocl.BallCutter(1, 5)
        path = ocl.Path()
        path.append(ocl.Line(P(-1, 5, -1), P(11, 5, -1)))
        pdc = ocl.PathDropCutter()
        pdc.setSTL(s)
        pdc.setCutter(c)
        pdc.setPath(path)
        pdc.minimumZ = -1
        self.assertEqual(pdc.getZ(), -1)
        pdc.setSampling(0.5)
        pdc.run()
        pts = pdc.getCLPoints()
        self.assertEqual(len(pts), 26)
        self.assertGreater(max(p.z for p in pts), 4.0)
        apdc = ocl.AdaptivePathDropCutter()
        apdc.setSTL(s)
        apdc.setCutter(c)
        apdc.setPath(path)
        apdc.setSampling(0.5)
        apdc.setMinSampling(0.01)
        apdc.run()
        self.assertGreater(len(apdc.getCLPoints()), 26)

    def test_helpers(self):
        f = ocl.LineCLFilter()
        for k in range(10):
            f.addCLPoint(ocl.CLPoint(k, 0, 0))
        f.setTolerance(0.01)
        f.run()
        self.assertEqual(len(f.getCLPoints()), 2)
        tsp = ocl.TSPSolver()
        for x, y in [(0, 0), (1, 0), (1, 1), (0, 1)]:
            tsp.addPoint(x, y)
        tsp.run()
        self.assertEqual(tsp.getOutput()[0], 0)
        self.assertAlmostEqual(tsp.getLength(), 4.0)
        cls = ocl.CutterLocationSurface(1.0)
        self.assertEqual(len(cls.getVertices()), 9)
        self.assertEqual(ocl.eps(), 2.0 ** -52)


if __name__ == "__main__":
    unittest.main()
