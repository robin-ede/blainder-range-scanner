"""
Pytest configuration: mock all Blender modules before any test imports
the range_scanner package.  This must run before any test file is collected.
"""

import math
import sys
import types
from unittest.mock import MagicMock


# ---------------------------------------------------------------------------
# Lightweight Vector replacement for mathutils.Vector
# ---------------------------------------------------------------------------

class MockVector:
    """Minimal mathutils.Vector substitute sufficient for validation tests."""

    def __init__(self, v):
        if isinstance(v, MockVector):
            self._v = v._v
        else:
            self._v = tuple(float(x) for x in v)

    # Attribute access used by alignment / evaluation code
    @property
    def x(self):
        return self._v[0]

    @property
    def y(self):
        return self._v[1]

    @property
    def z(self):
        return self._v[2]

    @property
    def length(self):
        return math.sqrt(sum(c * c for c in self._v))

    # Arithmetic
    def __add__(self, other):
        return MockVector(a + b for a, b in zip(self._v, other._v))

    def __sub__(self, other):
        return MockVector(a - b for a, b in zip(self._v, other._v))

    def __iter__(self):
        return iter(self._v)

    def __getitem__(self, i):
        return self._v[i]

    def __repr__(self):
        return "MockVector(%s)" % (self._v,)

    def __eq__(self, other):
        if isinstance(other, MockVector):
            return self._v == other._v
        return NotImplemented


# ---------------------------------------------------------------------------
# Stub Blender modules in sys.modules BEFORE anything imports range_scanner
# ---------------------------------------------------------------------------

def _make_mathutils():
    mod = types.ModuleType("mathutils")
    mod.Vector = MockVector
    bvhtree_mod = types.ModuleType("mathutils.bvhtree")
    bvhtree_mod.BVHTree = MagicMock()
    sys.modules["mathutils.bvhtree"] = bvhtree_mod
    mod.bvhtree = bvhtree_mod
    return mod


for _name in ["bpy", "bpy.ops", "bpy.props", "bpy.types", "bpy.utils",
              "bmesh", "bpy_extras", "bl_ui", "bl_operators"]:
    sys.modules.setdefault(_name, MagicMock())

_bpy_mock = sys.modules["bpy"]
_bpy_mock.path = MagicMock()
_bpy_mock.path.abspath = lambda x: x

_mathutils = _make_mathutils()
sys.modules["mathutils"] = _mathutils

# Stub the UI sub-package so range_scanner/__init__.py doesn't fail
sys.modules.setdefault("range_scanner.ui", MagicMock())
sys.modules.setdefault("range_scanner.ui.user_interface", MagicMock())


# ---------------------------------------------------------------------------
# Shared hit-object factory
# ---------------------------------------------------------------------------

def make_hit(
    x, y, z,
    noise_x=None, noise_y=None, noise_z=None,
    sensor_id="lidar",
    frame=1,
    distance=10.0,
    noise_distance=None,
    was_reflected=False,
):
    """Return a minimal hit-like namespace compatible with validation modules."""
    hit = types.SimpleNamespace()
    hit.location = MockVector((x, y, z))
    if noise_x is not None:
        hit.noiseLocation = MockVector((noise_x, noise_y, noise_z))
    else:
        hit.noiseLocation = None
    hit.sensor_id = sensor_id
    hit.frame = frame
    hit.distance = distance
    hit.noiseDistance = noise_distance
    hit.wasReflected = was_reflected
    return hit
