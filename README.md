# Edge AI-Driven Multi-Camera System for Adaptive Robot Speed Control in Safety-Critical Environments
 Simulation Implementation for "Edge AI-Driven Multi-Camera System for Adaptive Robot Speed Control in Safety-Critical Environments"


## Packages

### 1. UR3 (Noetic Devel)
- **Description**: ROS packages for controlling the Universal Robots UR3 robotic arm.
- **Source**: [ur3](https://github.com/cambel/ur3.git)

### 2. Realsense ROS (ROS1 Legacy)
- **Description**: Intel Realsense camera integration with ROS.
- **Source**: [realsense-ros](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy)
- **Source**: [realsense-ros-description](https://github.com/issaiass/realsense2_description)
  
### 3. YOLOv5 Segmentation
- **Description**: Real-time object detection algorithm implemented using PyTorch.
- **Source**: [yolov5](https://github.com/ultralytics/yolov5)
- **Segmentation Model**: [yolov5-segmentation](https://github.com/ultralytics/yolov5/releases/v7.0)


## Setting up ROS workspace

1. **Clone the repository**:

    ```bash
    cd ~/catkin_ws/src
    git clone https://github.com/IRaC-Lab/Robot-Speed-Control.git
    ```


## UR3 Package Install

1. **Installing Dependencies**:

    ```bash
    cd ~/catkin_ws
    rosinstall ~/catkin_ws/src/Robot-Speed-Control /opt/ros/noetic src/Robot-Speed-Control/ur3/dependencies.rosinstall
    sudo apt-get update
    rosdep fix-permissions
    rosdep update
    rosdep install --from-paths src --ignore-src --rosdistro=noetic -y
    ```

## IntelRealsense Package Install

**Step 1: Install the latest Intel® RealSense™ SDK 2.0**

1. **Register the server's public key**:

    ```bash
    sudo mkdir -p /etc/apt/keyrings
    curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
    ```
**Make sure apt HTTPS support is installed: sudo apt-get install apt-transport-https**
    
2. **Add the server to the list of repositories:**:

    ```bash
    echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
    sudo tee /etc/apt/sources.list.d/librealsense.list
    sudo apt-get update
    ```

3. **Install the libraries (see section below if upgrading packages)**:

    ```bash
    sudo apt-get install librealsense2-dkms
    sudo apt-get install librealsense2-utils
    ```
    
4. **Optionally install the developer and debug packages**:

    ```bash
    sudo apt-get install librealsense2-dev
    sudo apt-get install librealsense2-dbg
    ```
    
5. **Upgrading the Packages:**:

    ```bash
    sudo apt-get update
    sudo apt-get upgrade
    ```
    
**Step 2: Install Intel® RealSense™ ROS from Sources**

1. **Check Version of Package**:

    ```bash
    cd ~/catkin_ws/src/Robot-Speed-Control/realsense-ros
    git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`
    ```

2. **Install ddynamic_reconfigure package**:

    ```bash
    sudo apt-get update
    sudo apt-get install ros-noetic-ddynamic-reconfigure
    ```

3. **Build the packages**:

    ```bash
    catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
    catkin_make install
    ```

## Yolov5 Install

```bash
cd ~/catkin_ws/src/Robot-Speed-Control/yolov5
pip install -r requirements.txt
```
**Segmentation Model: (https://github.com/ultralytics/yolov5/releases/v7.0)**


## Build the packages

```bash
cd ~/catkin_ws/src/
catkin_make
```


## Running Gazebo and MoveIt

```bash
roslaunch ur_gripper_gazebo ur_gripper_85_person.launch
```
    
```bash
roslaunch ur_gripper_85_moveit_config start_moveit.launch
```

**If the MoveIt package is not installed**:
```bash
sudo apt update
sudo apt install ros-noetic-moveit
```

```bash
rosrun ur_control sim_ur_control.py
```

**When rosrun does not work**:
```bash
(in that directory) chmod +x (your script file)
```

## Running Operator Detection: YOLOv5n-seg, Alignment and Distance Measurment

```bash
cd catkin_ws/src/Robot-Speed-Control/yolov5/segment
python3 predict.py --conf 0.7
```
    
```bash
rosrun realsense2_camera align_depth_to_color.py
```

```bash
rosrun realsense2_camera depth_distance_seg.py
```

**When rosrun does not work**:
```bash
(in that directory) chmod +x (your script file)
```


## License

Each package within this repository is distributed under its respective license. Please refer to each package's original repository for specific licensing information.
