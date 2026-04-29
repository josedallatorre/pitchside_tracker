# Drone Autonomous Football Tracking Simulation

This project was developed for the course **Drone and Autonomous Robotic Systems**.
It simulates a drone that autonomously follows a football, adapting to factors such as speed and direction changes using computer vision and robotics frameworks.

---

## ️ Installation

You can run the project in two ways:

---

## Option 1: Run with Docker (Recommended)

This is the simplest method since all dependencies are pre-configured.

### Steps

```bash
cd src
sudo docker build -t josedallatorre/sjtu_drone:latest .
```

Builds a Docker image with all required dependencies.

```bash
sudo bash run_docker.sh
```

Starts the container and launches the simulation.

---

## Option 2: Manual Installation

If you prefer to run everything locally, follow these steps.

---

### 1. Install System Dependencies

```bash
sudo apt-get update \
    && apt-get install -y \
    wget curl unzip \
    lsb-release \
    mesa-utils \
    build-essential \
    && apt-get clean
```

Installs essential tools and libraries required for building and running the project.

---

### 2. Install Python and ML Libraries

```bash
curl -L https://bootstrap.pypa.io/get-pip.py -o get-pip.py
python3 get-pip.py
```

Installs `pip` for Python package management.

```bash
pip install ultralytics opencv-python
```

Installs computer vision and object detection tools.

```bash
pip uninstall -y numpy
pip install "numpy<2"
```
Ensures compatibility with other libraries.

```bash
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121
```

Installs PyTorch with CUDA support for GPU acceleration.

```bash
pip install lap pyyaml
```

Additional dependencies for tracking and configuration.

---

### 3. Install Gazebo and ROS 2

```bash
echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list \
    && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
    && apt-get update \
    && apt-get install -y \
    gazebo \
    ros-humble-gazebo-ros-pkgs \
    python3-colcon-common-extensions python3-rosdep --no-install-recommends \
    && apt-get clean
```

Installs the simulator (**Gazebo**) and ROS 2 integration packages.

```bash
rosdep init && rosdep update
```

Initializes ROS dependency management.

---

### 4. Download Gazebo Models

```bash
curl -L https://github.com/osrf/gazebo_models/archive/refs/heads/master.zip -o /tmp/gazebo_models.zip \
    && unzip /tmp/gazebo_models.zip -d /tmp \
    && mkdir -p ~/.gazebo/models/ \
    && mv /tmp/gazebo_models-master/* ~/.gazebo/models/ \
    && rm -r /tmp/gazebo_models.zip
```

Adds simulation assets (models, environments) for Gazebo.

---

### 5. Install ROS Dependencies

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

Installs all required ROS package dependencies.

---

### 6. Build the Workspace

```bash
source /opt/ros/humble/setup.bash
colcon build
```

Compiles the ROS 2 workspace.

---

## Run the Project

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py
```

Launches the simulation, including:

* Drone model
* Environment
* Autonomous tracking system

---