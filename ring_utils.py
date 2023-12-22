import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def convert_strings_to_float_array(strings):
    float_array = []
    for string in strings:
        # Remove newline characters and split the string into numbers
        numbers = string.replace('\n', '').split()
        # Convert each number to float and append to the list
        float_array.extend([float(num) for num in numbers])
    return np.array(float_array)

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


def print_camera_residuals_from_file(ring_params_file, bal_file):
    import general_utils

    # Load the ring parameters
    ring_params = load_ring_params(ring_params_file)

    with open(bal_file, 'r') as file:
        lines = file.readlines()

    # Parse the header to get the number of cameras, points, and observations
    header = lines[0]
    n_cameras, n_points, n_observations = map(int, header.split())

    # Calculate the start index for camera parameters
    camera_start_index = 1 + n_observations  # Observations first, then camera data

    for i in range(n_cameras):
        camera_data_start = camera_start_index + i * 9
        camera_data = lines[camera_data_start:camera_data_start + 9]

        camera_data = general_utils.convert_strings_to_float_array(camera_data)

        # Parse and extract the translation vector
        translation = camera_data[3:6]

        # Project the translation vector onto the ring and calculate the residual
        projected_translation = project_point_onto_ring(translation, ring_params)
        residual = np.linalg.norm(translation - projected_translation)

        print(f"Camera {i + 1}: Translation {translation}, Projected {projected_translation}, Residual: {residual:.5f}")


def update_bal_problem_with_ring_cameras(ring_params_file, old_bal_file, new_bal_file):
    import general_utils

    # Load the ring parameters
    ring_params = load_ring_params(ring_params_file)

    with open(old_bal_file, 'r') as file:
        lines = file.readlines()

    # Parse the header to get the number of cameras, points, and observations
    header = lines[0]
    n_cameras, n_points, n_observations = map(int, header.split())

    # Calculate the starting index for camera data (right after observations)
    camera_data_start_index = 1 + n_observations  # Start of camera data

    with open(new_bal_file, 'w') as file:
        file.write(header)

        # Write the observations unchanged
        for line in lines[1:camera_data_start_index]:
            file.write(line)

        # Process and write camera parameters
        for i in range(n_cameras):
            camera_start_index = camera_data_start_index + i * 9
            camera_data = lines[camera_start_index:camera_start_index + 9]

            camera_data = general_utils.convert_strings_to_float_array(camera_data)

            # Split and convert rotation and translation lines
            rotation = camera_data[:3]
            translation = camera_data[3:6]

            # Project translation onto the ring
            projected_translation = project_point_onto_ring(translation, ring_params)

            # Write updated camera parameters (rotation and projected translation)
            for angle in rotation:
              file.write(f"{angle}\n")
            for coord in projected_translation:
              file.write(f"{coord}\n")

            # Write the intrinsics and any other camera data unchanged
            for intrinsic in camera_data[6:]:
                file.write(str(intrinsic) + "\n")

        # Write the rest of the file unchanged (world points)
        for line in lines[camera_data_start_index + n_cameras * 10:]:
            file.write(line)



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



