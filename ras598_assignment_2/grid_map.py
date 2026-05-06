import cv2
import numpy as np
import yaml

def read_map_yaml(yaml_path):
    with open(yaml_path, "r") as f:
        return yaml.safe_load(f)

class GridMap:
    def __init__(self, image_path, yaml_path, cell_resolution=0.2, inflation_radius_m=0.65):
        self.cell_res  = cell_resolution
        self.inflate_r = inflation_radius_m
        data = read_map_yaml(yaml_path)
        self.img_res  = data["resolution"]
        self.origin_x = data["origin"][0]
        self.origin_y = data["origin"][1]
        self.world_size = 16.0

        raw = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if raw is None:
            raise FileNotFoundError(f"Cannot load map image: {image_path}")
        self.img_h, self.img_w = raw.shape

        # Black pixels = obstacles
        raw_pixel_grid = (raw < 128).astype(np.uint8)

        # Resize to A* grid resolution
        self.grid_w = int(self.world_size / cell_resolution)
        self.grid_h = int(self.world_size / cell_resolution)
        raw_resized = cv2.resize(
            raw_pixel_grid, (self.grid_w, self.grid_h),
            interpolation=cv2.INTER_NEAREST)

        # Inflate obstacles
        inflation_cells = int(np.ceil(inflation_radius_m / cell_resolution))
        kernel_size = 2 * inflation_cells + 1
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        self.grid = cv2.dilate(raw_resized, kernel, iterations=1)
        self.raw_grid = raw_resized

    def world_to_cell(self, wx, wy):
        """World (x,y) -> grid (col, row)."""
        col = int((wx - self.origin_x) / self.cell_res)
        row = int((self.world_size - (wy - self.origin_y)) / self.cell_res)
        col = max(0, min(self.grid_w - 1, col))
        row = max(0, min(self.grid_h - 1, row))
        return col, row

    def cell_to_world(self, col, row):
        """Grid (col, row) -> world center (x, y)."""
        wx = self.origin_x + col * self.cell_res + self.cell_res / 2.0
        wy = self.origin_y + self.world_size - row * self.cell_res - self.cell_res / 2.0
        return wx, wy
