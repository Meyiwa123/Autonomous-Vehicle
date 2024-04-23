import os
import cv2
import numpy as np
import ezdxf


def load_dxf(filename):
    doc = ezdxf.readfile(filename)
    msp = doc.modelspace()
    entities = list(msp)
    return entities


def rasterize_to_pgm(entities, grid_width, grid_height, cell_size):
    # Create an empty image representing the grid
    grid_image = np.zeros((grid_height, grid_width), dtype=np.uint8)

    # Loop through each cell in the grid
    for y in range(grid_height):
        for x in range(grid_width):
            # Check if the cell intersects with any entity representing obstacles
            if cell_intersects_obstacle(x, y, entities, cell_size):
                # Assign a higher intensity value for occupied cells
                grid_image[y, x] = 255
            else:
                # Assign a lower intensity value for free cells
                grid_image[y, x] = 0

    return grid_image


def cell_intersects_obstacle(x, y, entities, cell_size):
    # Calculate the coordinates of the cell
    cell_x1 = x * cell_size
    cell_y1 = y * cell_size
    cell_x2 = (x + 1) * cell_size
    cell_y2 = (y + 1) * cell_size

    # Check if any entity intersects with the cell
    for entity in entities:
        # Example: Check if a line entity intersects with the cell
        if entity.dxftype() == 'LINE':
            start = entity.dxf.start
            end = entity.dxf.end
            # Check if the line intersects with the cell boundaries
            if ((cell_x1 <= start[0] <= cell_x2 and cell_y1 <= start[1] <= cell_y2) or
                    (cell_x1 <= end[0] <= cell_x2 and cell_y1 <= end[1] <= cell_y2)):
                return True

    return False


def generate_yaml_file(map_width, map_height, resolution, origin_x, origin_y, yaml_filename):
    yaml_content = f"""image: {os.path.basename(yaml_filename).replace('.yaml', '.pgm')}
resolution: {resolution}
origin: [{origin_x}, {origin_y}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
negate_mode: true
"""

    with open(yaml_filename, 'w') as yaml_file:
        yaml_file.write(yaml_content)


# Define grid parameters
grid_width = 100
grid_height = 100
cell_size = 10  # Size of each cell in pixels

# Load DXF file
filename = os.path.join(os.path.dirname(__file__), 'carleton.dxf')
entities = load_dxf(filename)

# Rasterize to PGM
pgm_image = rasterize_to_pgm(entities, grid_width, grid_height, cell_size)

# Save as PGM file
pgm_filename = 'map.pgm'
cv2.imwrite(pgm_filename, pgm_image)

# Generate YAML file
resolution = 0.05  # Resolution of each grid cell in meters
origin_x = 0.0  # X coordinate of the map origin in meters
origin_y = 0.0  # Y coordinate of the map origin in meters
yaml_filename = 'map.yaml'
generate_yaml_file(grid_width, grid_height, resolution,
                   origin_x, origin_y, yaml_filename)
