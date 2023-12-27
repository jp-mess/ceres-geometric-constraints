import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def plot_cameras_and_point_cloud_center(center_point, cameras):
    """
    Plots the point cloud center, camera positions, and lines connecting them.
    :param center_point: Numpy array (x, y, z) for the center of the point cloud.
    :param cameras: List of camera dictionaries with 'extrinsic' matrices.
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the point cloud center
    ax.scatter(center_point[0], center_point[1], center_point[2], color='red', s=100, label='Point Cloud Center')

    # Plot the camera positions and lines connecting them to the center
    for idx, camera in enumerate(cameras):
        camera_position = camera['extrinsic'][:3, 3]
        ax.scatter(camera_position[0], camera_position[1], camera_position[2], color='blue', s=50)

        # Draw line between camera and center
        ax.plot([center_point[0], camera_position[0]], 
                [center_point[1], camera_position[1]], 
                [center_point[2], camera_position[2]], color='green')

        # Calculate and display Euclidean distance
        distance = np.linalg.norm(center_point - camera_position)
        mid_point = (center_point + camera_position) / 2
        ax.text(mid_point[0], mid_point[1], mid_point[2], f'{distance:.2f}', color='black')

    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    plt.legend()
    plt.show()
