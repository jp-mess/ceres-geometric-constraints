# 3D Computer Vision Simulation Environment

<p align="center">
  <img src="diagrams/car.png" alt="Car Image" style="width: 50%; height: 50%;"/>
</p>


## Author's Introduction

I made this repo to contain simulation and visualization scripts I use when messing around with point clouds, bundle adjustment, and camera models. I keep coming back to these scripts in my own work because they did turn out to be very useful (setting up a simulation environment that has correct right-hand rules, open3d frustum visualization, ceres encoding etc. is a pain in the butt). I wrote up the documentation using ChatGPT, and the paragraphs below have ChatGPT's distinctive tone. I did write the code, and while it's easy to follow I know I'm not winning any awards for it.  



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

## Functions

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

- **render_coordinates**
  - **Inputs**:
    - `coordinates`: A dictionary where each key is a point index, and the value is a tuple containing a pixel coordinate `(x, y)` and a depth value `z`.
    - `image_size`: A tuple `(height, width)` specifying the size of the output image.
    - `colors`: A list where each element corresponds to a color (as `[r, g, b]`) for each point index.
  - **Output**: A numpy array representing the rendered image, where each pixel's color is determined by the closest point projecting onto it.
  - **Description**: This function renders a 2D image from 3D point coordinates projected onto a 2D plane. It iterates over each 3D point, projects it onto the 2D plane using the provided pixel coordinates, and colors the corresponding pixel in the output image. If multiple points project onto the same pixel, the point with the smallest depth value (closest to the camera) determines the pixel's color. The function handles both horizontal and vertical flipping of the image to align with standard image coordinate systems.



### Running Experiments

To run an experiment, you will need to choose an appropriate function from `experiments.py`. The main function in the simulator will execute your selected experiment. For experiments involving bundle adjustment, the problem's encoding will be directed to the `problem_encodings` directory. A compressed point cloud of a BMW is provided in the `source-point-clouds` directory for convenience. You can also explore a variety of free point clouds available on [Sketchfab](https://sketchfab.com) for some larger and more interesting 3D models.

Please note that to run the simulator, you need to update the point cloud file path in `main.py`, as it is not set in `experiments.py`. Currently, everything inside `main.py` is hard-coded, and you run the main program with `python main.py`. Apologies for any inconvenience—this project was developed for personal use and convenience, and is not meant to be a rigorous, feature-complete framework.

