"""Converts probe world positions to competition grid sector IDs.

The ERC 2026 competition area is virtually divided into 1x1 m sectors.
Each sector is identified by a column letter and row number (e.g., "A2", "C4").
The probe detection system must report probe locations using these sector IDs.

Usage:
    mapper = GridMapper(config["grid"])
    sector = mapper.position_to_sector(x_m=1.5, y_m=0.8)
    # Returns "C1" or None if out of bounds
"""

from __future__ import annotations


class GridMapper:
    """Maps (x, y) positions in meters to alphanumeric grid sectors.

    Coordinate convention (matches Pixhawk LOCAL_POSITION_NED):
        - X axis (North) → mapped to COLUMNS (left to right on the grid).
        - Y axis (East)  → mapped to ROWS (bottom to top on the grid).

    The grid origin is the bottom-left corner of the grid, defined in
    config/mission_params.yaml. All input positions are relative to the
    takeoff pad center (the drone's local NED origin).
    """

    def __init__(self, grid_config: dict):
        """Initialize from the 'grid' section of mission_params.yaml.

        Args:
            grid_config: dict with keys:
                origin_x_m: X coordinate of the grid origin (bottom-left corner)
                            relative to the takeoff pad, in meters.
                origin_y_m: Y coordinate of the grid origin (bottom-left corner)
                            relative to the takeoff pad, in meters.
                cell_size_m: Side length of each square cell, in meters.
                columns: List of column labels (e.g., ["A", "B", "C", "D", "E", "F"]).
                rows: List of row labels (e.g., [1, 2, 3, 4, 5, 6]).
        """
        self.origin_x = float(grid_config["origin_x_m"])
        self.origin_y = float(grid_config["origin_y_m"])
        self.cell_size = float(grid_config["cell_size_m"])
        self.columns = [str(c) for c in grid_config["columns"]]
        self.rows = [str(r) for r in grid_config["rows"]]
        self.num_cols = len(self.columns)
        self.num_rows = len(self.rows)

    def position_to_sector(self, x_m: float, y_m: float) -> str | None:
        """Convert a world position to a grid sector ID.

        Args:
            x_m: X position relative to takeoff pad center, in meters (North).
            y_m: Y position relative to takeoff pad center, in meters (East).

        Returns:
            Sector ID string (e.g., "A2", "C4") if the position falls within
            the grid, or None if it's outside the grid boundaries.
        """
        # Translate from takeoff-pad-relative to grid-relative coordinates.
        # Example: if origin_x = -2.5 and the probe is at x = -1.0,
        # then grid_x = (-1.0 - (-2.5)) / 1.0 = 1.5 → column index 1.
        grid_x = (x_m - self.origin_x) / self.cell_size
        grid_y = (y_m - self.origin_y) / self.cell_size

        # Floor to get integer cell indices.
        # int() truncates toward zero, so we must handle negatives explicitly:
        # a position left of / below the grid origin gives a negative value.
        col_idx = int(grid_x) if grid_x >= 0 else -1
        row_idx = int(grid_y) if grid_y >= 0 else -1

        # Bounds check: index must be within [0, num_cols) and [0, num_rows).
        if not (0 <= col_idx < self.num_cols):
            return None
        if not (0 <= row_idx < self.num_rows):
            return None

        return f"{self.columns[col_idx]}{self.rows[row_idx]}"

    def get_grid_bounds(self) -> dict:
        """Return the world-coordinate boundaries of the grid.

        Useful for debugging and visualization.

        Returns:
            dict with keys: x_min, x_max, y_min, y_max (all in meters,
            relative to the takeoff pad).
        """
        return {
            "x_min": self.origin_x,
            "x_max": self.origin_x + self.num_cols * self.cell_size,
            "y_min": self.origin_y,
            "y_max": self.origin_y + self.num_rows * self.cell_size,
        }

    def __repr__(self) -> str:
        bounds = self.get_grid_bounds()
        return (
            f"GridMapper("
            f"cols={self.columns}, rows={self.rows}, "
            f"cell={self.cell_size}m, "
            f"x=[{bounds['x_min']}, {bounds['x_max']}], "
            f"y=[{bounds['y_min']}, {bounds['y_max']}])"
        )
