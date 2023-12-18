# 3D Computer Vision Simulation Environment

## Introduction

This repository provides a simulation environment for 3D computer vision alignment problems, specifically focusing on bundle adjustment. It serves as a straightforward setup for those interested in delving into computer vision algorithms without the complexity of building an experimental setup from the ground up. The environment includes functionality for noise modeling in camera views and an implementation of bundle adjustment using Ceres in C++, along with an encoding scheme to facilitate data exchange between Python scripts and the C++ optimization framework.

## Features

- **Camera View Sampling**: Tools for sampling camera views around a point cloud (`.ply` format).
- **Image Rendering**: Functionality to render point cloud views into images.
- **Point Cloud Retriangulation**: Capability for both pairwise and multiview DLT triangulation schemes.
- **Bundle Adjustment**: Simple Levenberg-Marquardt bundle adjustment is implemented and tested (with an easy encoding scheme for going back and forth between the python scripts, without using pybind11).

## Getting Started

Clone the repository to begin:

```bash
git clone https://github.com/your-username/your-repo-name.git
```bash


## Python Requirements

The Python dependencies for this project are listed in `requirements.txt`. To install these dependencies, run the following command in your Python environment:

```bash
pip install -r requirements.txt
```bash

