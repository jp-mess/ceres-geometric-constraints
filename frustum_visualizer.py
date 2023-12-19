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

    def create_wireframe_box(self, center, extents, rotation_matrix):
        corners = [
            center + np.dot(rotation_matrix, np.array([dx, dy, dz]))
            for dx in [-extents[0] / 2, extents[0] / 2]
            for dy in [-extents[1] / 2, extents[1] / 2]
            for dz in [-extents[2] / 2, extents[2] / 2]
        ]
        lines = [
            [0, 1], [0, 2], [1, 3], [2, 3],
            [4, 5], [4, 6], [5, 7], [6, 7],
            [0, 4], [1, 5], [2, 6], [3, 7]
        ]
        wireframe = o3d.geometry.LineSet()
        wireframe.points = o3d.utility.Vector3dVector(corners)
        wireframe.lines = o3d.utility.Vector2iVector(lines)
        wireframe.paint_uniform_color([1, 1, 1])  # White color
        return wireframe

    def create_frustum(self, camera_extrinsic, dim=0.2):
        camera_position = camera_extrinsic[:3, 3]
        direction = self.center_point - camera_position
        direction /= np.linalg.norm(direction)

        # Create rotation matrix
        up = np.array([0, 1, 0])
        right = np.cross(up, direction)
        right /= np.linalg.norm(right)
        up = np.cross(direction, right)
        rotation_matrix = np.column_stack((right, up, direction))

        extents = np.array([dim, dim, dim * 1.25])  # Making the depth 1.25 times the width/height
        wireframe_box = self.create_wireframe_box(camera_position, extents, rotation_matrix)
        return wireframe_box

    def visualize(self):
        visual_elements = [self.pcd]
        for camera in self.cameras:
            camera_marker = self.create_camera_marker(camera['extrinsic'][:3, 3])
            line = self.create_line(camera['extrinsic'])
            frustum = self.create_frustum(camera['extrinsic'])
            visual_elements.extend([camera_marker, line, frustum])

        vis = o3d.visualization.Visualizer()
        vis.create_window()
        for element in visual_elements:
            vis.add_geometry(element)

        opt = vis.get_render_option()
        opt.background_color = np.asarray([0.1, 0.1, 0.1])  # Charcoal grey background
        vis.run()
        vis.destroy_window()

