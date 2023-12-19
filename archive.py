import open3d as o3d
import numpy as np

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
        """ Creates a line between camera and point cloud center """
        camera_position = camera_extrinsic[:3, 3]
        distance = np.linalg.norm(self.center_point - camera_position)
        print(f"Distance from camera to center: {distance:.2f}")  # Log the distance

        line = o3d.geometry.LineSet()
        line.points = o3d.utility.Vector3dVector([self.center_point, camera_position])
        line.lines = o3d.utility.Vector2iVector([[0, 1]])
        line.paint_uniform_color([0, 1, 0])  # Green line
        return line

    def visualize(self):
        visual_elements = [self.pcd]
        for camera in self.cameras:
            camera_marker = self.create_camera_marker(camera['extrinsic'][:3, 3])
            line = self.create_line(camera['extrinsic'])
            visual_elements.extend([camera_marker, line])

        # Set up the visualizer
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        for element in visual_elements:
            vis.add_geometry(element)

        opt = vis.get_render_option()
        opt.background_color = np.asarray([0.1, 0.1, 0.1])  # Charcoal grey background
        vis.run()
        vis.destroy_window()

# Example usage remains the same
