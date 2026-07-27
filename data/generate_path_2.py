import cv2
import numpy as np

def find_square_corners(image_path):
    """
    Find the 4 corners of a closed black square in a PGM image.
    
    Args:
        image_path: Path to the PGM image file
    
    Returns:
        List of 4 corner points in order: top-left, top-right, bottom-right, bottom-left
    """
    # Read the image
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    
    if img is None:
        print(f"Error: Could not read image from {image_path}")
        return None
    
    # Invert the image if needed (assuming black square on white background)
    # If your square is black on white, you might want to invert it
    # For black square on white background, threshold to find the square
    _, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY_INV)
    
    # Alternative: If you have white square on black background, use:
    # _, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
    
    # Find contours
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        print("No contours found in the image.")
        return None
    
    # Find the largest contour (assuming it's the square)
    largest_contour = max(contours, key=cv2.contourArea)
    
    # Approximate the contour to a polygon
    epsilon = 0.02 * cv2.arcLength(largest_contour, True)
    approx = cv2.approxPolyDP(largest_contour, epsilon, True)
    
    # If we have 4 points, it's a quadrilateral
    if len(approx) == 4:
        corners = approx.reshape(4, 2)
        
        # Sort corners: top-left, top-right, bottom-right, bottom-left
        corners = sort_corners(corners)
        
        return corners
    else:
        print(f"Expected 4 corners, found {len(approx)} corners. Trying alternative method...")
        return find_corners_alternative(binary, largest_contour)

def sort_corners(corners):
    """
    Sort corner points in order: top-left, top-right, bottom-right, bottom-left
    """
    # Calculate the center of the corners
    center = np.mean(corners, axis=0)
    
    # Function to get angle from center
    def get_angle(point):
        angle = np.arctan2(point[1] - center[1], point[0] - center[0])
        return angle
    
    # Sort points by angle
    sorted_corners = sorted(corners, key=get_angle)
    
    # Reorder to start from top-left
    # Find the point with smallest x and y
    min_sum_idx = np.argmin([p[0] + p[1] for p in sorted_corners])
    sorted_corners = sorted_corners[min_sum_idx:] + sorted_corners[:min_sum_idx]
    
    return np.array(sorted_corners)

def find_corners_alternative(binary, contour):
    """
    Alternative method to find corners using min area rectangle
    """
    # Get the minimum area rectangle
    rect = cv2.minAreaRect(contour)
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    
    # Sort corners
    corners = sort_corners(box)
    
    return corners

def visualize_corners(image_path, corners):
    """
    Visualize the detected corners on the image with zoom and smaller dots
    """
    # Read the original image
    img = cv2.imread(image_path)
    
    if img is None:
        print("Error: Could not read image for visualization")
        return
    
    # Get original dimensions
    height, width = img.shape[:2]
    
    # Zoom in by 4x using resize
    zoom_factor = 4
    img_zoomed = cv2.resize(img, (width * zoom_factor, height * zoom_factor), 
                            interpolation=cv2.INTER_CUBIC)
    
    # Scale corner coordinates
    corners_scaled = corners * zoom_factor
    
    # Draw smaller corner dots (size 3 pixels)
    for i, corner in enumerate(corners_scaled):
        x, y = int(corner[0]), int(corner[1])
        # Smaller dot - radius 3
        cv2.circle(img_zoomed, (x, y), 3, (0, 255, 0), -1)
        # Smaller text
        cv2.putText(img_zoomed, f"C{i+1}", (x+8, y-8), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    
    # Draw thin lines connecting corners
    for i in range(4):
        x1, y1 = int(corners_scaled[i][0]), int(corners_scaled[i][1])
        x2, y2 = int(corners_scaled[(i+1)%4][0]), int(corners_scaled[(i+1)%4][1])
        cv2.line(img_zoomed, (x1, y1), (x2, y2), (255, 0, 0), 1)
    
    # Create a window that can be resized
    cv2.namedWindow('Square Corners - Zoomed', cv2.WINDOW_NORMAL)
    
    # If the zoomed image is too large, set a maximum display size
    max_display_width = 1200
    max_display_height = 900
    
    if img_zoomed.shape[1] > max_display_width or img_zoomed.shape[0] > max_display_height:
        # Resize for display while maintaining aspect ratio
        display_scale = min(max_display_width / img_zoomed.shape[1], 
                           max_display_height / img_zoomed.shape[0])
        display_width = int(img_zoomed.shape[1] * display_scale)
        display_height = int(img_zoomed.shape[0] * display_scale)
        img_display = cv2.resize(img_zoomed, (display_width, display_height))
        cv2.imshow('Square Corners - Zoomed', img_display)
    else:
        cv2.imshow('Square Corners - Zoomed', img_zoomed)
    
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    return img_zoomed

def save_result(image_path, corners, zoom_factor=4):
    """
    Save the image with marked corners (zoomed version)
    """
    img = cv2.imread(image_path)
    
    if img is None:
        print("Error: Could not read image for saving")
        return
    
    height, width = img.shape[:2]
    
    # Zoom in
    img_zoomed = cv2.resize(img, (width * zoom_factor, height * zoom_factor), 
                            interpolation=cv2.INTER_CUBIC)
    
    # Scale corners
    corners_scaled = corners * zoom_factor
    
    # Draw smaller corner dots
    for corner in corners_scaled:
        x, y = int(corner[0]), int(corner[1])
        cv2.circle(img_zoomed, (x, y), 3, (0, 255, 0), -1)
    
    # Draw thin lines connecting corners
    for i in range(4):
        x1, y1 = int(corners_scaled[i][0]), int(corners_scaled[i][1])
        x2, y2 = int(corners_scaled[(i+1)%4][0]), int(corners_scaled[(i+1)%4][1])
        cv2.line(img_zoomed, (x1, y1), (x2, y2), (255, 0, 0), 1)
    
    # Save the zoomed version
    output_path = image_path.replace('.pgm', '_corners_zoomed.jpg')
    cv2.imwrite(output_path, img_zoomed)
    print(f"Result saved to: {output_path}")
    
    # Also save a version with the original size
    output_path_original = image_path.replace('.pgm', '_corners.jpg')
    img_original = cv2.imread(image_path)
    for corner in corners:
        x, y = int(corner[0]), int(corner[1])
        cv2.circle(img_original, (x, y), 2, (0, 255, 0), -1)
    for i in range(4):
        x1, y1 = int(corners[i][0]), int(corners[i][1])
        x2, y2 = int(corners[(i+1)%4][0]), int(corners[(i+1)%4][1])
        cv2.line(img_original, (x1, y1), (x2, y2), (255, 0, 0), 1)
    cv2.imwrite(output_path_original, img_original)
    print(f"Original size result saved to: {output_path_original}")

def main():
    # Replace with your image path
    image_path = '/Users/b/Documents/GitHub/Cartographer_test1/data/square_map_filled.pgm'
    
    # Find corners
    corners = find_square_corners(image_path)
    
    if corners is not None:
        print("Detected corners (in order: top-left, top-right, bottom-right, bottom-left):")
        for i, corner in enumerate(corners):
            print(f"Corner {i+1}: ({corner[0]:.1f}, {corner[1]:.1f})")
        
        # Visualize the results with zoom
        visualize_corners(image_path, corners)
        
        # Save the result
        save_result(image_path, corners)
        
        # Print corner coordinates in different formats
        print("\nCorner coordinates (x, y):")
        print(f"TL: ({corners[0][0]:.1f}, {corners[0][1]:.1f})")
        print(f"TR: ({corners[1][0]:.1f}, {corners[1][1]:.1f})")
        print(f"BR: ({corners[2][0]:.1f}, {corners[2][1]:.1f})")
        print(f"BL: ({corners[3][0]:.1f}, {corners[3][1]:.1f})")
        
        # Calculate square properties
        width = np.sqrt((corners[1][0] - corners[0][0])**2 + (corners[1][1] - corners[0][1])**2)
        height = np.sqrt((corners[2][0] - corners[1][0])**2 + (corners[2][1] - corners[1][1])**2)
        print(f"\nSquare dimensions:")
        print(f"Width: {width:.1f} pixels")
        print(f"Height: {height:.1f} pixels")

if __name__ == "__main__":
    main()