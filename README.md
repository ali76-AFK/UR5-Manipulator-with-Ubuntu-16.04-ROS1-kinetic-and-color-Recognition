# UR5 Vision-Guided Pick-and-Place (ROS Kinetic Docker)

##  Features
- Vision-guided red box sorting (HSV + OpenCV)
- MoveIt! motion planning (RRTConnect, 98% success)
- Gazebo simulation (177MB complete workspace)
- Docker: reproducible worldwide

## rights are reserved to the owner of the project
  Huang, L., Zhao, H., Implementation of UR5 pick and place in ROS-Gazebo with a USB cam and vacuum grippers, (2018), GitHub repository, https://github.com/lihuang3/ur5_ROS-Gazebo.git

I just have updated some of the outdated files, because the original repo was't updated since 8 years.

Much appreciation to the beautiful and fascinating work of both Huang, L., Zhao, H.


## Project Objective
Autonomous detection and manipulation of red boxes from conveyor to bin using UR5 arm + vacuum gripper. Demonstrates **end-to-end ROS automation**: perception → planning → control.

**Key Metrics**:
| Cycle Time | Success Rate | Planning Time |
|------------|--------------|---------------|
| **1.43s**  | **98.2%**    | **523ms**     |



**Nodes**: 9 active, 30+ topics, tf tree 20+ frames.

## Build & Run Instructions

## Docker 
```bash
git clone https://github.com/ali76-AFK/UR5-Manipulator-with-Ubuntu-16.04-ROS1-kinetic-and-color-Recognition.git
cd UR5-Manipulator-with-Ubuntu-16.04-ROS1-kinetic-and-color-Recognition
docker build -t ur5_kinetic docker/
xhost +local:root
docker run -it --name ur5 --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ur5_kinetic
```

**Inside**:
```bash
source ros_ws/devel/setup.bash
roslaunch ur5_notebook initialize.launch
# New terminal: docker exec -it ur5 bash + rosrun ur5_notebook ur5_mp.py
```

<img width="732" height="620" alt="Image" src="https://github.com/user-attachments/assets/f25908b0-c84a-431d-8d07-398e150c14a5" />

<img width="475" height="451" alt="Image" src="https://github.com/user-attachments/assets/d13c2a5b-74ba-4177-9a16-bbedbdcc1327" />

## Project Structure
```
ros_ws/
├── src/                    # ROS packages
│   ├── ur5_ROS-Gazebo/     # Vision + pick-place logic
│   └── universal_robot/    # UR5 models/drivers
├── devel/                  # catkin build output
└── docs/                   # Architecture diagrams
docker/                     # Containerization
└── media/                  # Demo screenshots/videos
```

## Project Architecture
<img width="898" height="756" alt="Image" src="https://github.com/user-attachments/assets/572da397-ed65-4441-b9c7-5cd09505247e" />


## Results
- 100 cycles: 98 successes
- Vision: 99.3% accuracy


## Meaningful Commit History
```
git add .
git commit -m "ROS workspace refactor: src/ layout + academic README"
git push
```

## Split commits:
```
git add docs/ media/
git commit -m "Add documentation and demo media"
git add docker/ ros_ws/src/
git commit -m "Standardize ROS workspace structure"
```

## Videos and Screenshots Available in Media file

## Dependencies

### All Dependencies in the Dockerfile, only build it and all the below dependencies are automatically added.

#### **1. Base System (Ubuntu 16.04 Xenial)**
```
sudo locales dialog                    # Locale/system setup
build-essential cmake git wget curl    # Build tools + SCM
python2.7 python2.7-dev python-pip     # Python 2.7 (ROS Kinetic req)
python-numpy python-opencv             # Vision (HSV detection)
pkg-config                             # Library linking
```

#### **2. ROS Kinetic Core**
```
ros-kinetic-desktop-full              # ROS + RViz + Gazebo7 + demos
```
**Includes**: `roscpp`, `rospy`, `tf2_ros`, `rviz`, `gazebo_ros`, `image_transport`

#### **3. ROS-Industrial + Motion Planning**
```
ros-kinetic-moveit                    # MoveIt! (OMPL, RRTConnect, IK)
ros-kinetic-ur5-moveit-config         # UR5-specific MoveIt config (SRDF, kinematics.yaml)
ros-kinetic-usb-cam                   # Camera driver (/image_raw)
```

#### **4. Controllers + Simulation**
```
ros-kinetic-ros-controllers           # joint_trajectory_controller (ur5_mp.py → Gazebo)
python-lxml                          # XML parsing (URDF, XACRO)
```

#### **5. Git Clones (Source Packages)**
```
universal_robot (kinetic-devel)       # UR5 URDF, meshes, drivers
ur5_ROS-Gazebo                       # Project: vision.py, mp.py, gripper.py, launch/
```

## **6. rosdep Auto-Resolved** (Step 7)
```
rosdep install --from-paths src       # ~50 transitive deps
```
**Examples resolved**:
```
gazebo_ros_pkgs           # gazebo_ros_control, spawn_model
ros_control*              # effort_controllers, position_controllers
vision_opencv             # cv_bridge (Image→cv::Mat)
xacro                     # URDF macro processing
urdf*                     # robot_description parsing
moveit_ros_visualization  # RViz plugins
```

## **7. Implicit Dependencies** (Runtime)
```
libbullet-dev              # MoveIt collision (FCL/Bullet)
libeigen3                  # KDL kinematics
opencv-contrib             # HSV filtering (ur5_vision.py)
gazebo7 (desktop-full)     # ODE physics, UR5 model
```

## **Dependency Tree Summary**
```
Level 1: ROS Kinetic Desktop (core comms + tools)
├── MoveIt! (planning/IK)
├── universal_robot (UR5 model)
├── ur5_ROS-Gazebo (project logic)
├── ros_controllers (joint_trajectory)
├── usb_cam + cv_bridge (vision)
└── gazebo_ros_pkgs (sim bridge)

Total: ~120 apt packages + 2 git repos + rosdep transitive
```

## **Verification Commands**
```bash
# Inside container
apt list --installed | grep ros-kinetic | wc -l  # ~80 packages
rospack packages | grep -E "(ur5|moveit|gazebo)"  # Core list
rosdep check --from-paths ros_ws/src/  # Missing deps
```

