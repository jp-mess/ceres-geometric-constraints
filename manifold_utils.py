import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D



def load_ring_params(file_path):
    ring_params = {}
    with open(file_path, 'r') as file:
        for line in file:
            key, value = line.strip().split(': ')
            ring_params[key] = np.fromstring(value, sep=',')
    return ring_params

def project_point_onto_ring(point, ring_params):
    center = ring_params['center']
    normal = ring_params['normal']
    radius = ring_params['radius'][0]

    #print('center', center)
    #print('normal', normal)
    #print('radius', radius)

    # Translate the point to the ring's coordinate system
    translated_point = point - center

    # Project the point onto the ring's plane
    point_on_plane = translated_point - np.dot(translated_point, normal) * normal

    # Normalize to get the direction from the center to the point on the plane
    direction = point_on_plane / np.linalg.norm(point_on_plane)

    # Scale the direction to match the radius of the ring
    point_on_ring = center + direction * radius

    return point_on_ring



def print_cameras_distance_to_ring(cameras, ring_params_path):
    ring_params = load_ring_params(ring_params_path)

    for idx, camera in enumerate(cameras):
        camera_position = camera['extrinsic'][:3, 3]
        projected_position = project_point_onto_ring(camera_position, ring_params)
        
        distance = np.linalg.norm(camera_position - projected_position)
        radius = np.linalg.norm(camera_position - ring_params["center"])
        print(f"Camera {idx + 1}: Original Position {camera_position}, Projected Position {projected_position}, Distance to Center: {radius:.5f}, Distance to Ring: {distance:.5f}")


def main():
    ring_params = load_ring_params("manifold_encodings/ring_params.txt")
    random_points = np.random.uniform(-100, 100, (100, 3))
    projected_points = np.array([project_point_onto_ring(point, ring_params) for point in random_points])

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(random_points[:, 0], random_points[:, 1], random_points[:, 2], color='blue', label='Original Points')
    ax.scatter(projected_points[:, 0], projected_points[:, 1], projected_points[:, 2], color='red', label='Projected Points')
    ax.legend()

    plt.show()

if __name__ == "__main__":
    main()



