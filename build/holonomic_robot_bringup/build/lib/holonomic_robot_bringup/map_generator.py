#!/usr/bin/env python3
"""
Generate a static occupancy grid map from an 8-point polygon.
Saves as PGM image and YAML metadata for use with map_server and AMCL.
"""

import numpy as np
from PIL import Image, ImageDraw
import yaml
import os


def generate_polygon_map(polygon_points, resolution=0.05, output_dir='.'):
    """
    Generate an occupancy grid map from polygon points.
    
    Args:
        polygon_points: List of (x, y) tuples defining the polygon boundary
        resolution: Map resolution in meters/pixel
        output_dir: Directory to save map files
    """
    # Convert points to numpy array
    points = np.array(polygon_points)
    
    # Find bounds
    min_x, min_y = points.min(axis=0)
    max_x, max_y = points.max(axis=0)
    
    # Add padding
    padding = 1.0  # meters
    min_x -= padding
    min_y -= padding
    max_x += padding
    max_y += padding
    
    # Calculate map dimensions
    width = int((max_x - min_x) / resolution)
    height = int((max_y - min_y) / resolution)
    
    # Create empty map (free space = 254, occupied = 0, unknown = 205)
    map_data = np.full((height, width), 254, dtype=np.uint8)
    
    # Create PIL image for drawing
    img = Image.fromarray(map_data)
    draw = ImageDraw.Draw(img)
    
    # Convert polygon points to pixel coordinates
    pixel_points = []
    for x, y in polygon_points:
        px = int((x - min_x) / resolution)
        py = height - int((y - min_y) / resolution)  # Flip Y axis (image coordinates)
        pixel_points.append((px, py))
    
    # Draw filled polygon (inside is free = 254)
    draw.polygon(pixel_points, fill=254, outline=None)
    
    # Draw thick boundary walls (occupied = 0)
    line_width = int(0.15 / resolution)  # 15cm thick walls
    for i in range(len(pixel_points)):
        p1 = pixel_points[i]
        p2 = pixel_points[(i + 1) % len(pixel_points)]
        draw.line([p1, p2], fill=0, width=line_width)
    
    # Fill outside with unknown (205)
    # First, create a mask for the polygon
    mask = Image.new('L', (width, height), 0)
    mask_draw = ImageDraw.Draw(mask)
    mask_draw.polygon(pixel_points, fill=255)
    
    # Convert back to array
    map_data = np.array(img)
    mask_data = np.array(mask)
    
    # Set outside to unknown
    map_data[mask_data == 0] = 205
    
    # Save map image (PGM format required by map_server)
    map_img = Image.fromarray(map_data, mode='L')
    map_file = os.path.join(output_dir, 'polygon_map.pgm')
    map_img.save(map_file)
    
    # Create YAML metadata
    yaml_data = {
        'image': 'polygon_map.pgm',
        'resolution': float(resolution),
        'origin': [float(min_x), float(min_y), 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196
    }
    
    yaml_file = os.path.join(output_dir, 'polygon_map.yaml')
    with open(yaml_file, 'w') as f:
        yaml.dump(yaml_data, f, default_flow_style=False)
    
    print(f"Map generated successfully:")
    print(f"  Size: {width}x{height} pixels")
    print(f"  Resolution: {resolution} m/pixel")
    print(f"  Origin: ({min_x:.2f}, {min_y:.2f}, 0.0)")
    print(f"  Files:")
    print(f"    - {map_file}")
    print(f"    - {yaml_file}")


if __name__ == '__main__':
    # Define 8-point polygon (same as in environment_markers and fake_lidar)
    polygon = [
        (-1.5, -1.0),   # Point 0: Bottom left
        (-1.5, 1.0),    # Point 1: Top left
        (-0.5, 1.5),    # Point 2: Top left corner
        (0.5, 1.5),     # Point 3: Top right corner
        (1.5, 1.0),     # Point 4: Top right
        (1.5, -1.0),    # Point 5: Bottom right
        (0.5, -1.5),    # Point 6: Bottom right corner
        (-0.5, -1.5),   # Point 7: Bottom left corner
    ]
    
    # Get output directory from command line or use default
    import sys
    if len(sys.argv) > 1:
        output_dir = sys.argv[1]
    else:
        # Default to maps directory in the package
        output_dir = os.path.dirname(__file__)
        output_dir = os.path.join(os.path.dirname(output_dir), 'maps')
    
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Generate map with 5cm resolution
    generate_polygon_map(polygon, resolution=0.05, output_dir=output_dir)
