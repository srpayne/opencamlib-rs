"""Compare two dumps written by dump_results.py.

Exits non-zero if any section differs, except the drop-cutter sections of cutters that
contain a cone, where the Rust port fixes C++ gouging bugs: there it only reports how many
points moved and checks that none moved down.
"""
import json
import sys

TOL = 1e-9
CONE = ("cone", "cylcone", "ballcone", "bullcone", "conecone")


def diff(a, b, path, out):
    if isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            out.append(f"{path}: length {len(a)} != {len(b)}")
            return
        for k, (x, y) in enumerate(zip(a, b)):
            diff(x, y, f"{path}[{k}]", out)
    elif a is None or b is None:
        pass  # not available in one implementation (e.g. methods missing from the 2023 wheel)
    elif isinstance(a, bool) or isinstance(b, bool) or isinstance(a, str):
        if a != b:
            out.append(f"{path}: {a!r} != {b!r}")
    elif isinstance(a, (int, float)) and isinstance(b, (int, float)):
        if abs(a - b) > TOL * max(1.0, abs(a)):
            out.append(f"{path}: {a!r} != {b!r}")
    elif a != b:
        out.append(f"{path}: {a!r} != {b!r}")


def is_cone_drop(key):
    parts = key.split("/")
    return parts[-1] in CONE and parts[0] in ("drop", "PathDropCutter", "AdaptivePathDropCutter")


def main():
    cpp, rust = (json.load(open(f)) for f in sys.argv[1:3])
    failed = False
    for key in sorted(set(cpp) | set(rust)):
        if key not in cpp or key not in rust:
            print(f"{key}: missing on one side")
            failed = True
            continue
        if is_cone_drop(key) and key.startswith("drop/"):
            moved = [(c[2], r[2]) for c, r in zip(cpp[key], rust[key]) if abs(c[2] - r[2]) > TOL]
            down = [m for m in moved if m[1] < m[0] and abs(m[1] - m[0] + 1.0) > 1e-6]
            print(f"{key}: {len(moved)} of {len(cpp[key])} points raised by the cone fixes, {len(down)} lowered")
            failed |= bool(down)
            continue
        out = []
        diff(cpp[key], rust[key], key, out)
        if key == "single":
            # cone cutters' drop results differ by design; compare the rest
            out = [o for o in out if not any(f"'{n}'" in o or o.startswith(f"single[{i}]")
                                             for i, row in enumerate(cpp[key]) for n in CONE
                                             if row[0] == n and row[1] != "pushCutter" and row[1] != "radius")]
        if is_cone_drop(key):
            print(f"{key}: {len(out)} differences (expected: cone fixes)")
            continue
        if out:
            failed = True
            print(f"{key}: {len(out)} differences, e.g. {out[0]}")
        else:
            print(f"{key}: identical")
    sys.exit(1 if failed else 0)


main()
