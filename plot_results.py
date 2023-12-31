import sys
import open3d as o3d
from frustum_visualizer import PointCloudCameraVisualizer
from geometry_utils import load_bal_problem_file
import ring_utils

def main(bal_file, ring_file=None):
    cameras, points = load_bal_problem_file(bal_file)

    # Create an Open3D point cloud object
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    center_point = None

    ring_dict = None
    if ring_file:
        ring_dict = ring_utils.load_ring_params(ring_file)

    visualizer = PointCloudCameraVisualizer(pcd, cameras, center_point, ring_dict, draw_green_directions=True)
    visualizer.visualize()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python visualize_bundle_adjustment.py <bal_file> [<ring_file>]")
        sys.exit(1)

    bal_file = sys.argv[1]
    ring_file = sys.argv[2] if len(sys.argv) > 2 else None

    main(bal_file, ring_file)

