import cv2
import numpy as np

def find_square_corners(image_path, dark_thresh=50, free_thresh=250,
                        wall_distance=2.0, step=0.25, spacing=3.0, debug=True):
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Could not load image: {image_path}")

    _, wall_mask = cv2.threshold(img, dark_thresh, 255, cv2.THRESH_BINARY_INV)
    _, free_mask = cv2.threshold(img, free_thresh, 255, cv2.THRESH_BINARY)

    if debug:
        print(f"wall pixels: {np.count_nonzero(wall_mask)}, "
              f"free(white) pixels: {np.count_nonzero(free_mask)}")

    kernel = np.ones((3, 3), np.uint8)
    wall_mask = cv2.morphologyEx(wall_mask, cv2.MORPH_CLOSE, kernel, iterations=1)

    contours, _ = cv2.findContours(wall_mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        cv2.imwrite("debug_thresh.png", wall_mask)
        raise ValueError("No contours found. Saved mask to debug_thresh.png")

    largest = max(contours, key=cv2.contourArea)
    rect = cv2.minAreaRect(largest)
    corners = cv2.boxPoints(rect).astype("float32")
    corners = order_corners(corners)

    free_space = cv2.bitwise_not(wall_mask)
    dist_map = cv2.distanceTransform(free_space, cv2.DIST_L2, 5)

    center = corners.mean(axis=0)
    inner_corners = np.array([
        move_corner_inside(pt, center, dist_map, free_mask,
                           wall_distance, step)
        for pt in corners
    ], dtype="float32")

    # NEW: generate wall-hugging dots along each edge
    edge_dots = generate_edge_dots(inner_corners, center, dist_map, free_mask,
                                   wall_distance, spacing, step)

    return img, corners, inner_corners, edge_dots, dist_map, free_mask


def move_corner_inside(corner, center, dist_map, free_mask, target_dist, step):
    """Walk corner toward center until on white free space AND far enough."""
    h, w = dist_map.shape
    vec = center - corner
    total = np.linalg.norm(vec)
    if total == 0:
        return corner
    unit = vec / total

    pos = corner.astype("float32").copy()
    traveled = 0.0
    while traveled <= total:
        xi, yi = int(round(pos[0])), int(round(pos[1]))
        if 0 <= xi < w and 0 <= yi < h:
            if free_mask[yi, xi] > 0 and dist_map[yi, xi] >= target_dist:
                return pos
        pos = pos + unit * step
        traveled += step
    return center.astype("float32")


def push_to_wall(point, outward, center, dist_map, free_mask,
                 target_dist, step, max_push=50.0):
    """
    Move `point` in the `outward` direction (toward wall) and stop at the
    last position that is still white free space AND >= target_dist clearance.
    This makes the dot hug the wall at the desired clearance.
    """
    h, w = dist_map.shape
    pos = point.astype("float32").copy()
    best = pos.copy()
    pushed = 0.0
    while pushed <= max_push:
        xi, yi = int(round(pos[0])), int(round(pos[1]))
        if 0 <= xi < w and 0 <= yi < h:
            if free_mask[yi, xi] > 0 and dist_map[yi, xi] >= target_dist:
                best = pos.copy()      # still valid -> remember it
            else:
                break                  # crossed clearance limit -> stop
        else:
            break
        pos = pos + outward * step
        pushed += step
    return best


def generate_edge_dots(corners, center, dist_map, free_mask,
                       target_dist, spacing, step):
    """
    Sample dots along each edge of the (inner) square, then push each one
    outward toward the wall so it sits at target_dist clearance.
    """
    dots = []
    n = len(corners)
    for i in range(n):
        p0 = corners[i]
        p1 = corners[(i + 1) % n]

        edge_vec = p1 - p0
        edge_len = np.linalg.norm(edge_vec)
        if edge_len == 0:
            continue
        edge_unit = edge_vec / edge_len

        # outward normal (perpendicular), pointing away from center
        normal = np.array([-edge_unit[1], edge_unit[0]], dtype="float32")
        mid = (p0 + p1) / 2.0
        if np.dot(normal, mid - center) < 0:
            normal = -normal

        # place dots at `spacing` intervals, skipping the exact corners
        num = int(edge_len // spacing)
        for k in range(1, num):
            base = p0 + edge_unit * (k * spacing)
            dot = push_to_wall(base, normal, center, dist_map, free_mask,
                               target_dist, step)
            dots.append(dot)

    return np.array(dots, dtype="float32") if dots else np.empty((0, 2), "float32")


def order_corners(pts):
    pts = np.array(pts, dtype="float32")
    rect = np.zeros((4, 2), dtype="float32")
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    return rect


if __name__ == "__main__":
    # image_path = "/Users/b/Documents/GitHub/Cartographer_test1/data/pure_square_map.pgm"
    image_path = "/Users/b/Documents/GitHub/Cartographer_test1/data/square_map_filled.pgm" 
    WALL_DISTANCE = 3.0           # clearance from wall (pixels)
    SPACING = 1.0                 # gap between edge dots (pixels)

    img, corners, inner_corners, edge_dots, dist_map, free_mask = \
        find_square_corners(image_path, wall_distance=WALL_DISTANCE,
                            spacing=SPACING)

    labels = ["Top-Left", "Top-Right", "Bottom-Right", "Bottom-Left"]
    print(f"\nInner corners (white, >= {WALL_DISTANCE}px from wall):")
    for label, (x, y) in zip(labels, inner_corners):
        xi, yi = int(round(x)), int(round(y))
        print(f"  {label:12s}: ({x:.1f}, {y:.1f})  "
              f"wall_dist={dist_map[yi, xi]:.2f}  val={img[yi, xi]}")

    print(f"\nGenerated {len(edge_dots)} edge dots (spacing {SPACING}px):")
    for x, y in edge_dots:
        xi, yi = int(round(x)), int(round(y))
        print(f"  ({x:.1f}, {y:.1f})  wall_dist={dist_map[yi, xi]:.2f}  "
              f"val={img[yi, xi]}")

    scale = 12
    vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    vis = cv2.resize(vis, None, fx=scale, fy=scale,
                     interpolation=cv2.INTER_NEAREST)

    # original corners (red)
    for x, y in corners:
        cv2.circle(vis, (int(x * scale), int(y * scale)), 5, (0, 0, 255), -1)

    # inner square outline (green) + inner corners
    inner_scaled = (inner_corners * scale).astype(np.int32)
    cv2.polylines(vis, [inner_scaled], isClosed=True,
                  color=(0, 255, 0), thickness=1)
    for x, y in inner_corners:
        cv2.circle(vis, (int(x * scale), int(y * scale)), 5, (0, 255, 0), -1)

    # edge dots hugging the wall (blue)
    for x, y in edge_dots:
        cv2.circle(vis, (int(x * scale), int(y * scale)), 4, (255, 0, 0), -1)

    cv2.imwrite("corners_result.png", vis)
    print("\nSaved visualization to corners_result.png")
    cv2.imshow("red=orig  green=inner corners  blue=wall dots", vis)
    cv2.waitKey(0)
    cv2.destroyAllWindows()