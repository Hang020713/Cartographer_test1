#!/usr/bin/env python3
import yaml
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from PIL import Image
import sys

def visualize_saved_map(yaml_path):
    # Load the YAML file
    with open(yaml_path, 'r') as f:
        map_metadata = yaml.safe_load(f)
    
    # Extract metadata
    image_path = map_metadata['image']
    resolution = map_metadata['resolution']
    origin = map_metadata['origin']  # [x, y, yaw]
    
    # Load the image
    # The image path is relative to the YAML file
    import os
    image_dir = os.path.dirname(yaml_path)
    image_full_path = os.path.join(image_dir, image_path)
    
    img = Image.open(image_full_path).convert('L')  # Convert to grayscale
    data = np.array(img)
    
    # ROS occupancy grid convention: 0=free, 100=occupied, -1=unknown
    # PGM files are typically 0=black(occupied), 255=white(free)
    # Convert to ROS convention for display
    data_ros = np.where(data < 128, 100, 0)  # Simple threshold
    data_ros = np.where(data == 0, -1, data_ros)  # Unknown if needed
    
    height, width = data.shape
    origin_x, origin_y, yaw = origin
    
    # Create figure
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    
    # ----- Plot 1: Map with origin (real-world) -----
    extent = [
        origin_x,
        origin_x + width * resolution,
        origin_y,
        origin_y + height * resolution
    ]
    
    ax1.imshow(data_ros, cmap='gray', origin='lower', extent=extent, vmin=-1, vmax=100)
    ax1.set_title('Map with Origin Point')
    ax1.set_xlabel('X (meters)')
    ax1.set_ylabel('Y (meters)')
    
    # Mark origin (0,0)
    ax1.scatter(0, 0, color='red', s=200, marker='o', zorder=5, label='Origin (0,0)')
    ax1.scatter(origin_x, origin_y, color='blue', s=100, marker='x', zorder=5, 
               label='Map Bottom-Left')
    
    circle = Circle((0, 0), 0.3, color='red', fill=False, linestyle='--', linewidth=2)
    ax1.add_patch(circle)
    
    ax1.annotate(f'Origin (0,0)', xy=(0, 0), xytext=(0.3, 0.3),
                fontsize=10, color='red', fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    
    # ----- Plot 2: Pixel coordinates -----
    ax2.imshow(data, cmap='gray', origin='lower')
    ax2.set_title('Map in Pixel Coordinates')
    ax2.set_xlabel('Pixel X')
    ax2.set_ylabel('Pixel Y')
    ax2.scatter(0, 0, color='red', s=200, marker='o', zorder=5, label='Pixel (0,0)')
    ax2.legend()
    
    # Display metadata
    metadata_text = (
        f'Map Info:\n'
        f'Width: {width} pixels\n'
        f'Height: {height} pixels\n'
        f'Resolution: {resolution:.3f} m/pixel\n'
        f'Origin: ({origin_x:.3f}, {origin_y:.3f}) m\n'
        f'Yaw: {yaw:.3f} rad\n'
        f'Map size: {width*resolution:.2f} x {height*resolution:.2f} m'
    )
    fig.text(0.5, 0.02, metadata_text, ha='center', fontsize=10,
            bbox=dict(boxstyle='round,pad=0.5', facecolor='lightgray', alpha=0.8))
    
    plt.tight_layout()
    plt.show()
    
    print(f"\n{'='*50}")
    print(f"MAP ORIGIN INFORMATION")
    print(f"{'='*50}")
    print(f"Origin (x, y): ({origin_x:.3f}, {origin_y:.3f}) meters")
    print(f"Yaw: {yaw:.3f} radians ({np.degrees(yaw):.1f} degrees)")
    print(f"Resolution: {resolution:.3f} m/pixel")
    print(f"Image dimensions: {width} x {height} pixels")
    print(f"Physical dimensions: {width*resolution:.2f} x {height*resolution:.2f} meters")
    print(f"\n💡 The origin (0,0) in world coordinates is at the bottom-left")
    print(f"   corner of the map image, located at ({origin_x:.3f}, {origin_y:.3f})")
    print("   in real-world coordinates.")
    print("="*50)

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 visualize_map.py <path_to_map.yaml>")
        print("Example: python3 visualize_map.py ~/map.yaml")
        sys.exit(1)
    
    visualize_saved_map(sys.argv[1])