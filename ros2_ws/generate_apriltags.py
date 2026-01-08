#!/usr/bin/env python3
"""
Generate AprilTag images for Gazebo simulation
AprilTag patterns are pre-computed designs from the tag36h11 family
"""
import numpy as np
import cv2
import os

# AprilTag 36h11 patterns (simplified representation)
# Real tags use complex error correction, these are visual approximations
# For production, use official AprilTag generator from the AprilTag library

def create_apriltag_image(tag_id, output_path, size=1000):
    """Create a basic AprilTag-like pattern"""
    
    # Create white background with black border
    img = np.ones((size, size), dtype=np.uint8) * 255
    
    # Add black outer border (2 modules wide)
    border_width = size // 10
    img[0:border_width, :] = 0
    img[size-border_width:size, :] = 0
    img[:, 0:border_width] = 0
    img[:, size-border_width:size] = 0
    
    # Inner data area (simplified pattern based on tag ID)
    inner_start = border_width * 2
    inner_end = size - border_width * 2
    module_size = (inner_end - inner_start) // 6
    
    # Tag 0 pattern (simple pattern for demonstration)
    if tag_id == 0:
        patterns = [
            [1, 0, 1, 0, 1, 0],
            [0, 1, 1, 0, 0, 1],
            [1, 1, 0, 0, 1, 0],
            [0, 0, 1, 1, 0, 1],
            [1, 0, 0, 1, 1, 0],
            [0, 1, 0, 1, 0, 1],
        ]
    # Tag 1 pattern
    elif tag_id == 1:
        patterns = [
            [0, 1, 0, 1, 0, 1],
            [1, 0, 0, 1, 1, 0],
            [0, 1, 1, 0, 0, 1],
            [1, 0, 1, 0, 1, 0],
            [0, 1, 0, 1, 1, 0],
            [1, 1, 0, 0, 1, 1],
        ]
    else:
        patterns = [[1, 0] * 3 for _ in range(6)]
    
    # Draw pattern
    for row in range(6):
        for col in range(6):
            if patterns[row][col] == 0:  # Black module
                y1 = inner_start + row * module_size
                y2 = inner_start + (row + 1) * module_size
                x1 = inner_start + col * module_size
                x2 = inner_start + (col + 1) * module_size
                img[y1:y2, x1:x2] = 0
    
    # Add white inner border
    inner_border = border_width
    img[border_width:inner_border, border_width:size-border_width] = 255
    img[size-inner_border:size-border_width, border_width:size-border_width] = 255
    img[border_width:size-border_width, border_width:inner_border] = 255
    img[border_width:size-border_width, size-inner_border:size-border_width] = 255
    
    # Save image
    cv2.imwrite(output_path, img)
    print(f"✅ Created AprilTag {tag_id}: {output_path}")
    return True

def main():
    print("Generating AprilTag images for Gazebo simulation...")
    print("Note: These are simplified patterns. For production, use official AprilTag generator.")
    print()
    
    # Get workspace path
    workspace = os.path.dirname(os.path.abspath(__file__))
    
    # Generate tags
    for tag_id in [0, 1]:
        model_dir = os.path.join(workspace, f"src/courier_world/models/apriltag_{tag_id}")
        os.makedirs(model_dir, exist_ok=True)
        
        output_path = os.path.join(model_dir, "tag.png")
        create_apriltag_image(tag_id, output_path, size=1000)
    
    print()
    print("✅ AprilTag generation complete!")
    print("Tags are located in: src/courier_world/models/apriltag_0/ and apriltag_1/")

if __name__ == "__main__":
    main()
