import numpy as np

def parse_bal_file(file_path):
    with open(file_path, 'r') as file:
        num_cameras, num_points, num_observations = map(int, file.readline().split())

        # Skip observation lines
        for _ in range(num_observations):
            file.readline()

        # Read camera parameters
        camera_data = [list(map(float, file.readline().split())) for _ in range(num_cameras)]

        # Read 3D point coordinates
        point_data = [list(map(float, file.readline().split())) for _ in range(num_points)]

    return np.array(camera_data), np.array(point_data)

def compute_average_error(truth, estimate):
    return np.mean(np.linalg.norm(truth - estimate, axis=1))

def main(ground_truth_file, solution_file):
    ground_truth_cameras, ground_truth_points = parse_bal_file(ground_truth_file)
    solution_cameras, solution_points = parse_bal_file(solution_file)

    camera_error = compute_average_error(ground_truth_cameras, solution_cameras)
    point_error = compute_average_error(ground_truth_points, solution_points)

    print(f"Average Camera Estimation Error: {camera_error}")
    print(f"Average World Point Error: {point_error}")

if __name__ == "__main__":
    import sys

    if len(sys.argv) != 3:
        print("Usage: python error_checker.py <ground_truth_bal_file> <solution_bal_file>")
        sys.exit(1)

    main(sys.argv[1], sys.argv[2])

