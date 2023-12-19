import open3d as o3d
import numpy as np

class PointCloudCameraVisualizer:
    def __init__(self, pcd, cameras, center_point):
        self.pcd = pcd
        self.cameras = cameras
        self.center_point = center_point

    def create_camera_marker(self, location, size=0.05):
        return o3d.geometry.TriangleMesh.create_sphere(radius=size).translate(location)

    def create_line(self, camera_extrinsic):
        camera_position = camera_extrinsic[:3, 3]
        line = o3d.geometry.LineSet()
        line.points = o3d.utility.Vector3dVector([self.center_point, camera_position])
        line.lines = o3d.utility.Vector2iVector([[0, 1]])
        line.paint_uniform_color([0, 1, 0])  # Green line
        return line

    def create_frustum(self, extrinsic, intrinsic, scale=1):  
        scale = 10
        """ Creates a frustum for visualizing the camera's field of view """
        frustum = o3d.geometry.LineSet()
        fx, fy, cx, cy = intrinsic[0, 0], intrinsic[1, 1], intrinsic[0, 2], intrinsic[1, 2]
        near = 0.01 * scale
        far = 0.1 * scale

        # Compute the 4 corners of the far plane
        corners_far = np.array([
            [cx - fx, cy - fy, far],
            [cx + fx, cy - fy, far],
            [cx + fx, cy + fy, far],
            [cx - fx, cy + fy, far]
        ]) / fx

        # Compute the 4 corners of the near plane
        corners_near = corners_far * (near / far)
        corners = np.vstack([[[0, 0, 0]], corners_near, corners_far])  # Camera position + 8 corners

        # Transform corners from camera space to world space
        corners_homogenous = np.hstack([corners, np.ones((9, 1))])  # Convert to homogeneous coordinates
        corners_world = (extrinsic @ corners_homogenous.T).T[:, :3]

        # Define lines of the frustum
        lines = [
            [0, 1], [0, 2], [0, 3], [0, 4],  # Camera to near plane corners
            [1, 5], [2, 6], [3, 7], [4, 8],  # Near to far plane corners
            [5, 6], [6, 7], [7, 8], [8, 5]   # Far plane edges
        ]

        frustum.points = o3d.utility.Vector3dVector(corners_world)
        frustum.lines = o3d.utility.Vector2iVector(lines)
        frustum.paint_uniform_color([1, 1, 1])  # Frustum color set to white

        return frustum

    def visualize(self):
        visual_elements = [self.pcd]
        for camera in self.cameras:
            camera_marker = self.create_camera_marker(camera['extrinsic'][:3, 3])
            line = self.create_line(camera['extrinsic'])
            frustum = self.create_frustum(camera['extrinsic'], camera['intrinsic'])
            visual_elements.extend([camera_marker, line, frustum])

        vis = o3d.visualization.Visualizer()
        vis.create_window()
        for element in visual_elements:
            vis.add_geometry(element)

        opt = vis.get_render_option()
        opt.background_color = np.asarray([0.1, 0.1, 0.1])  # Charcoal grey background
        vis.run()
        vis.destroy_window()

# Example usage remains the same
