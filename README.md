# dynamixel_hardware_interface_demos

This repository collects example packages and resources for working with Dynamixel hardware using the ros2_control framework. 

## Overview
This repository is intended to help users get started with Dynamixel hardware integration in ROS 2. It provides example configurations, launch files, and scripts to demonstrate how to use the `dynamixel_hardware_interface` with ros2_control and controller_manager.

## Included Packages

- [dynamixel_hardware_interface_example](dynamixel_hardware_interface_example/README.md)
  - Example package with configuration files, launch files, and scripts for setting up and running a Dynamixel-based robot system using ros2_control.
- [dynamixel_hardware_interface_example_1](dynamixel_hardware_interface_example_1/README.md)
  - Example package demonstrating a dual Dynamixel system (two buses) with separate ros2_control configurations and a dedicated launch file for running both systems together.

## Getting Started

### Prerequisites
- **For the recommended Docker environment:**
  - [Docker Engine installed](https://docs.docker.com/engine/install/)
  - Dynamixel hardware (e.g., motors, U2D2, etc.)
- **For local installation (alternative):**
  - ROS 2 (Jazzy or later recommended)
  - Dynamixel hardware (e.g., motors, U2D2, etc.)

## Docker Development Environment (Recommended)

A ready-to-use Docker environment is provided in the [`docker/`](docker/) directory. This setup allows you to build and run the workspace in a containerized ROS 2 environment, making it easy to get started without installing dependencies directly on your system.

> **Recommended:** Use the Docker environment for the most consistent and reproducible setup.

### Usage

From the root of the repository, use the provided script to build and run the container:

```bash
./docker/container.sh <command>
```

#### Available Commands
- `help`   : Show usage instructions.
- `start`  :
  - Sets up X11 forwarding for GUI applications (if `DISPLAY` is set).
  - Copies the `99-u2d2.rules` file for U2D2 device permissions to `/etc/udev/rules.d/` (requires sudo).
  - Reloads udev rules.
  - Pulls the latest Docker images and starts the container using `docker compose`.
- `enter`  :
  - Opens an interactive bash shell inside the running container (with X11 forwarding if available).
- `stop`   :
  - Prompts for confirmation, then stops and removes the running container using `docker compose down`.

#### Example Usage
```bash
./docker/container.sh start   # Start the container
./docker/container.sh enter   # Enter the running container
./docker/container.sh stop    # Stop and remove the container
./docker/container.sh help    # Show help message
```

**Notes:**
- X11 forwarding is set up for GUI applications if the `DISPLAY` variable is set.
- Sudo privileges are required to copy udev rules for U2D2 device support.
- For advanced options and more details, see comments in [`docker/container.sh`](docker/container.sh).

## Local Installation (Alternative)

If you prefer not to use Docker, you can install and build the workspace locally:

```bash
cd <your_ros2_ws>/src
git clone https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface_demos.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```
