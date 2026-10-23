# Test data

`tux.stl` and `demo.stl` are binary copies of `stl/gnu_tux_mod.stl` and `stl/demo.stl`
from the [OpenCAMLib](https://github.com/aewallin/opencamlib) repository (LGPL-2.1).
The coordinates are the same 32-bit floats as in the original ASCII files.

The `.txt` files are reference results from the C++ library (Python wheel
`opencamlib==2023.1.11`), generated with:

```bash
pip install opencamlib==2023.1.11
python tests/gen_reference.py path/to/opencamlib/stl tests/data
```

The core algorithms in the C++ repository have not changed since that release.
