"""Unit tests for the GridMapper utility.

Run with: python3 -m pytest tests/test_grid_mapper.py -v
Or without pytest: python3 tests/test_grid_mapper.py
"""

from __future__ import annotations

import sys
import os

# Ensure the project root is on the path so 'src' can be imported.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from src.utils.grid_mapper import GridMapper


# Default grid config matching config/mission_params.yaml.
DEFAULT_GRID_CONFIG = {
    "origin_x_m": -2.5,
    "origin_y_m": -0.5,
    "cell_size_m": 1.0,
    "columns": ["A", "B", "C", "D", "E", "F"],
    "rows": [1, 2, 3, 4, 5, 6],
}


def make_mapper(config=None) -> GridMapper:
    return GridMapper(config or DEFAULT_GRID_CONFIG)


# ---------------------------------------------------------------
# Basic mapping tests
# ---------------------------------------------------------------

def test_origin_cell():
    """A position at the grid origin should map to the first cell."""
    m = make_mapper()
    # origin is at (-2.5, -0.5). A point at (-2.0, 0.0) is inside cell (0,0).
    assert m.position_to_sector(-2.0, 0.0) == "A1"


def test_center_of_grid():
    """A position in the center of the grid."""
    m = make_mapper()
    # Grid spans x: [-2.5, 3.5], y: [-0.5, 5.5].
    # Center is approximately (0.5, 2.5) → column C (idx 2+), row 3 (idx 2+).
    # x=0.5: (0.5 - (-2.5)) / 1.0 = 3.0 → idx 3 → "D"
    # y=2.5: (2.5 - (-0.5)) / 1.0 = 3.0 → idx 3 → "4"
    assert m.position_to_sector(0.5, 2.5) == "D4"


def test_last_cell():
    """A position in the last valid cell (F6)."""
    m = make_mapper()
    # Last cell: column F (idx 5), row 6 (idx 5).
    # x must be in [2.5, 3.5), y must be in [4.5, 5.5).
    assert m.position_to_sector(3.0, 5.0) == "F6"


def test_cell_boundaries_left_edge():
    """A position exactly on a cell boundary goes to the cell with the lower index."""
    m = make_mapper()
    # x=-2.5 is exactly at the grid origin → idx 0.
    # y=-0.5 is exactly at the grid origin → idx 0.
    assert m.position_to_sector(-2.5, -0.5) == "A1"


def test_cell_boundaries_inner():
    """A position on an inner cell boundary."""
    m = make_mapper()
    # x=-1.5: (-1.5 - (-2.5)) / 1.0 = 1.0 → idx 1 → "B"
    # y=0.5: (0.5 - (-0.5)) / 1.0 = 1.0 → idx 1 → "2"
    assert m.position_to_sector(-1.5, 0.5) == "B2"


# ---------------------------------------------------------------
# Out-of-bounds tests
# ---------------------------------------------------------------

def test_out_of_bounds_left():
    """Position left of the grid (x < origin_x)."""
    m = make_mapper()
    assert m.position_to_sector(-3.0, 0.0) is None


def test_out_of_bounds_right():
    """Position right of the grid (x >= origin_x + num_cols * cell_size)."""
    m = make_mapper()
    # Grid x ends at -2.5 + 6*1.0 = 3.5.
    assert m.position_to_sector(3.5, 0.0) is None


def test_out_of_bounds_below():
    """Position below the grid (y < origin_y)."""
    m = make_mapper()
    assert m.position_to_sector(0.0, -1.0) is None


def test_out_of_bounds_above():
    """Position above the grid (y >= origin_y + num_rows * cell_size)."""
    m = make_mapper()
    # Grid y ends at -0.5 + 6*1.0 = 5.5.
    assert m.position_to_sector(0.0, 5.5) is None


def test_far_out_of_bounds():
    """Position way outside the grid."""
    m = make_mapper()
    assert m.position_to_sector(100.0, 100.0) is None
    assert m.position_to_sector(-100.0, -100.0) is None


# ---------------------------------------------------------------
# Edge cases
# ---------------------------------------------------------------

def test_takeoff_pad_center():
    """The takeoff pad is at (0, 0) in local NED. It should map to a valid sector."""
    m = make_mapper()
    # x=0: (0 - (-2.5)) / 1.0 = 2.5 → idx 2 → "C"
    # y=0: (0 - (-0.5)) / 1.0 = 0.5 → idx 0 → "1"
    result = m.position_to_sector(0.0, 0.0)
    assert result is not None
    assert result == "C1"


def test_custom_grid_config():
    """GridMapper works with a different config."""
    config = {
        "origin_x_m": 0.0,
        "origin_y_m": 0.0,
        "cell_size_m": 0.5,
        "columns": ["X", "Y", "Z"],
        "rows": ["a", "b"],
    }
    m = make_mapper(config)
    assert m.position_to_sector(0.25, 0.25) == "Xa"
    assert m.position_to_sector(1.0, 0.5) == "Zb"
    assert m.position_to_sector(1.5, 0.0) is None  # Outside


def test_grid_bounds():
    """get_grid_bounds returns correct values."""
    m = make_mapper()
    bounds = m.get_grid_bounds()
    assert bounds["x_min"] == -2.5
    assert bounds["x_max"] == 3.5   # -2.5 + 6*1.0
    assert bounds["y_min"] == -0.5
    assert bounds["y_max"] == 5.5   # -0.5 + 6*1.0


def test_repr():
    """__repr__ doesn't crash and contains useful info."""
    m = make_mapper()
    r = repr(m)
    assert "GridMapper" in r
    assert "1.0m" in r


# ---------------------------------------------------------------
# Run without pytest
# ---------------------------------------------------------------

if __name__ == "__main__":
    test_functions = [
        v for k, v in sorted(globals().items())
        if k.startswith("test_") and callable(v)
    ]
    passed = 0
    failed = 0
    for fn in test_functions:
        try:
            fn()
            print(f"  PASS {fn.__name__}")
            passed += 1
        except AssertionError as e:
            print(f"  FAIL {fn.__name__}: {e}")
            failed += 1
        except Exception as e:
            print(f"  ERROR {fn.__name__}: {type(e).__name__}: {e}")
            failed += 1

    print(f"\n{passed} passed, {failed} failed out of {passed + failed} tests.")
    sys.exit(1 if failed else 0)
