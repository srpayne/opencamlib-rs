"""OpenCAMLib toolpath algorithms, implemented in Rust.

Mirrors the API of the original C++ ``opencamlib`` package, so both
``import opencamlib as ocl`` and ``from opencamlib import ocl`` work.
"""

from . import ocl
from .ocl import *  # noqa: F401,F403

__version__ = ocl.version()
