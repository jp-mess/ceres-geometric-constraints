# 3D Computer Vision Simulation Environment

<p align="center">
  <img src="diagrams/car.png" alt="Car Image" style="width: 50%; height: 50%;"/>
</p>


## Author's Introduction

I made this repo to contain simulation and visualization scripts I use when messing around with point clouds, bundle adjustment, and camera models. I keep coming back to these scripts in my own work because they did turn out to be very useful (setting up a simulation environment that has correct right-hand rules, open3d frustum visualization, ceres encoding, et cetera). I wrote up the documentation using ChatGPT, and the paragraphs below have ChatGPT's distinctive tone. Please steal, mutilate and share this code for your own purposes and advancement (I do not need to be cited). 



## ChatGPT's Introduction

This repository provides a simulation environment for 3D computer vision alignment problems, specifically focusing on bundle adjustment. It serves as a straightforward setup for those interested in delving into computer vision algorithms without the complexity of building an experimental setup from the ground up. The environment includes functionality for noise modeling in camera views and an implementation of bundle adjustment using Ceres in C++, along with an encoding scheme to facilitate data exchange between Python scripts and the C++ optimization framework.

## Features

- **Camera View Sampling**: Tools for sampling camera views around a point cloud (`.ply` format).
- **Image Rendering**: Functionality to render point cloud views into images.
- **Point Cloud Retriangulation**: Capability for both pairwise and multiview DLT triangulation schemes.
- **Bundle Adjustment**: The implementation utilizes a Levenberg-Marquardt algorithm, mirroring the encoding style found in the [Ceres Solver examples](https://github.com/ceres-solver/ceres-solver). This code includes support functions for experimental exploration with manifolds through local parameterization.

## Python Requirements

The Python dependencies for this project are listed in `requirements.txt`. To install these dependencies, run the following command in your Python environment:

```bash
pip install -r requirements.txt
```

## Using the Python Simulation Environment

<p align="center">
  <img src="diagrams/images_of_cars.png" alt="Pictures of Cars" style="width: 100%; height: 100%;"/>
</p>

### Overview

The Python component of the simulator, primarily encapsulated within `experiments.py`, comprises various functions for setting up and conducting 3D computer vision experiments. These functions facilitate the creation of tailored camera models with specific extrinsic and intrinsic parameters, designed to work with individual point clouds.

### Key Requirements

- **Y-Axis Up-Direction**: It's essential for all point clouds used in experiments to have a 'y-axis' up-direction. This orientation is crucial for ensuring the accuracy of camera models and subsequent processes.
- **Experiment Functions**: The functions in `experiments.py` each represent a unique experimental setup, specifically crafted for different point clouds. Designed to be intuitive, these functions guide users through the setup process.

## Python Functions (everything but BA optimization)

- **make_cameras**
  - **Inputs**:
    - `point_cloud_center`: A numpy array representing the 3D center of the point cloud.
    - `radius`: A float specifying the radius of the hemisphere around the point cloud where cameras are placed.
    - `up_direction`: A string indicating the up direction ('y' for upward).
    - `num_cameras`: An integer specifying the number of cameras to generate.
    - `close`: A boolean indicating whether cameras should be placed close to each other (optional).
  - **Output**: A list of pose matrices, where each matrix represents the extrinsic parameters of a camera.
  - **Description**: This function generates a specified number of cameras distributed uniformly over a hemisphere centered around the point cloud. Each camera is oriented towards the center of the point cloud.

- **make_cameras_on_ring**
  - **Inputs**:
    - `point_cloud_center`: A numpy array representing the 3D center of the point cloud.
    - `radius`: A float specifying the radius of the ring around the point cloud where cameras are placed.
    - `up_direction`: A string indicating the up direction ('y' for upward).
    - `num_cameras`: An integer specifying the number of cameras to generate.
  - **Output**: A list of pose matrices, where each matrix represents the extrinsic parameters of a camera.
  - **Description**: This function generates a specified number of cameras distributed uniformly along a ring at a fixed elevation angle around the point cloud. The elevation angle is randomly chosen between 60 and 90 degrees from the horizontal plane. Each camera is oriented towards the center of the point cloud.

- **rasterize**
  - **Inputs**:
    - `cameras`: A list of camera objects, where each camera object contains 'extrinsic' and 'intrinsic' parameters.
    - `indices_to_project`: A list of indices specifying which points in the point cloud to project.
    - `points`: A numpy array of 3D points in the point cloud.
    - `colors`: A list of colors associated with each point in the point cloud.
  - **Output**: A dictionary named `correspondences`, where each key is a point index, and the value is a list of tuples. Each tuple contains the pixel coordinates and the depth of the point as seen from each camera.
  - **Description**: This function rasterizes a 3D point cloud onto the image planes of multiple cameras. For each camera, it projects the specified points onto the camera's image plane using the camera's extrinsic and intrinsic parameters. The function calculates the pixel coordinates and depth of each point from the camera's perspective. The result is a dictionary of correspondences that maps each point index to its pixel locations and depths in all the cameras.

- **render_coordinates**
  - **Inputs**:
    - `coordinates`: A dictionary where each key is a point index, and the value is a tuple containing a pixel coordinate `(x, y)` and a depth value `z`.
    - `image_size`: A tuple `(height, width)` specifying the size of the output image.
    - `colors`: A list where each element corresponds to a color (as `[r, g, b]`) for each point index.
  - **Output**: A numpy array representing the rendered image, where each pixel's color is determined by the closest point projecting onto it.
  - **Description**: This function renders a 2D image from 3D point coordinates projected onto a 2D plane. It iterates over each 3D point, projects it onto the 2D plane using the provided pixel coordinates, and colors the corresponding pixel in the output image. If multiple points project onto the same pixel, the point with the smallest depth value (closest to the camera) determines the pixel's color. The function handles both horizontal and vertical flipping of the image to align with standard image coordinate systems.

- **retriangulate**
  - **Inputs**:
    - `cameras`: A list of camera dictionaries, where each dictionary contains a 'camera' key with a 4x4 pose matrix and a 'name' key with a string identifier.
    - `correspondences`: A dictionary mapping point indices to a list of tuples. Each tuple contains pixel coordinates `(x, y)` and depth information.
    - `points`: A numpy array of 3D points in the point cloud.
    - `noise_scale`: A float specifying the amount of Gaussian noise to add to each pixel observation (default is 0.0).
    - `pairwise`: A boolean indicating whether to perform pairwise triangulation (default is True). If False, performs multiview triangulation.
    - `save_dir`: A string specifying the directory to save the retriangulated point cloud (optional).
  - **Output**: A numpy array of retriangulated 3D points.
  - **Description**: This function retriangulates a point cloud from given correspondences using the Direct Linear Transform (DLT) method. It supports both pairwise triangulation (creating a world point for each view edge) and multiview triangulation (creating a single world point for all cameras observing the same point). The function can add Gaussian noise to pixel observations, compute the average retriangulation error, and optionally save the retriangulated point cloud as a `.ply` file. The output is a numpy array of the retriangulated points.

## PointCloudCameraVisualizer Class

The `PointCloudCameraVisualizer` class is designed for visual debugging of 3D point clouds and camera positions. It allows the visualization of a point cloud along with camera frustums, camera markers, and lines indicating the camera's view direction. This class is particularly useful for verifying camera placements and orientations in 3D space.

### Methods
- **`__init__(self, pcd, cameras, center_point)`**
  - Initializes the visualizer with a point cloud (`pcd`), a list of camera dictionaries (`cameras`), and a center point (`center_point`) around which the cameras are positioned.
- **`create_camera_marker(self, location, size=0.05)`**
  - Creates a spherical marker to represent the camera's position. `location` specifies the camera's 3D position, and `size` sets the sphere's radius.
- **`create_line(self, camera_extrinsic)`**
  - Generates a line between the camera position (extracted from `camera_extrinsic`) and the center point of the point cloud. This line helps visualize the camera's view direction.
- **`create_wireframe_box(self, center, extents, rotation_matrix)`**
  - Creates a wireframe box representing a camera's frustum. The box's center, size (extents), and orientation (rotation_matrix) are specified by the camera's position and view direction.
- **`create_frustum(self, camera_extrinsic, dim=0.2)`**
  - Constructs a frustum for a camera using its extrinsic parameters. The `dim` parameter controls the size of the frustum.
- **`visualize(self)`**
  - Renders the point cloud, camera markers, view lines, and frustums in a 3D visualization window. The background color of the window is set to charcoal grey.

### Usage Example
To use `PointCloudCameraVisualizer`, create an instance by passing a point cloud, a list of cameras, and a center point. Then, call the `visualize` method to open a 3D visualization window:

```python
pcd = o3d.geometry.PointCloud()  # Your point cloud
cameras = [{'extrinsic': camera_extrinsic, 'intrinsic': camera_intrinsic}]  # List of cameras
center_point = np.array([x, y, z])  # Center point around which cameras are positioned

visualizer = PointCloudCameraVisualizer(pcd, cameras, center_point)
visualizer.visualize()


### Running Experiments

To run an experiment, you will need to choose an appropriate function from `experiments.py`. The main function in the simulator will execute your selected experiment. For experiments involving bundle adjustment, the problem's encoding will be directed to the `problem_encodings` directory. A compressed point cloud of a BMW is provided in the `source-point-clouds` directory for convenience. You can also explore a variety of free point clouds available on [Sketchfab](https://sketchfab.com) for some larger and more interesting 3D models.

Please note that to run the simulator, you need to update the point cloud file path in `main.py`, as it is not set in `experiments.py`. Currently, everything inside `main.py` is hard-coded, and you run the main program with `python main.py`. Apologies for any inconvenience—this project was developed for personal use and convenience, and is not meant to be a rigorous, feature-complete framework.

