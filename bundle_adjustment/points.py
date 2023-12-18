import numpy as np
import open3d as o3d
import sys

def load_point_cloud_from_bal_file(filename):
    with open(filename, 'r') as file:
        # Read the header to get the number of cameras, points, and observations
        num_cameras, num_points, num_observations = map(int, file.readline().split())

        # Skip over the observations
        for _ in range(num_observations):
            file.readline()

        # Skip over the camera parameters
        for _ in range(num_cameras * 9):
            file.readline()

        # Read the 3D points
        points = []
        for _ in range(num_points):
            x = float(file.readline().strip())
            y = float(file.readline().strip())
            z = float(file.readline().strip())
            points.append([x, y, z])

    return np.array(points)

def main():
    if len(sys.argv) != 2:
        print("Usage: python script.py <input_file>")
        sys.exit(1)

    filename = sys.argv[1]
    point_cloud_data = load_point_cloud_from_bal_file(filename)

    # Create Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud_data)

    # Save point cloud to a file or visualize it
    o3d.io.write_point_cloud("output_point_cloud.ply", pcd)
    #o3d.visualization.draw_geometries([pcd])  # Visualize the point cloud

if __name__ == "__main__":
    main()
