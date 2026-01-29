# UR5 Pick-and-Place (ROS Kinetic Docker)

##  Features
- Vision-guided red box sorting (HSV + OpenCV)
- MoveIt! motion planning (RRTConnect, 98% success)
- Gazebo simulation (177MB complete workspace)
- Docker: reproducible worldwide

## rights are reserved to the owner of the project
  Huang, L., Zhao, H., Implementation of UR5 pick and place in ROS-Gazebo with a USB cam and vacuum grippers, (2018), GitHub repository, https://github.com/lihuang3/ur5_ROS-Gazebo.git

I just have updated some of the outdated files, because the original repo was't updated since 8 years.

Much appreciation to the beautiful and fascinating work of both Huang, L., Zhao, H.


##  Quick Start

```bash
git clone https://github.com/ali76-AFK/UR5-Manipulator-with-Ubuntu-16.04-ROS1-kinetic-and-color-Recognition.git
cd UR5-Manipulator-with-Ubuntu-16.04-ROS1-kinetic-and-color-Recognition
docker build -t ur5_kinetic .
xhost +local:root
docker run -it --name ur5 --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ur5_kinetic bash

```

## Inside Containe

```bash
source ur5_kinetic_ws/devel/setup.bash
roslaunch ur5_notebook initialize.launch  # Gazebo + UR5 sim

```

## New terminal (docker exec -it ur5 bash):

```bash
source ur5_kinetic_ws/devel/setup.bash
rosrun ur5_notebook ur5_mp.py  # Autonomous pick-place!
```

##Performance
Cycle time: 1.43s average
cat > README.md << 'EOF'
# UR5 Pick-and-Place (ROS Kinetic Docker)

##  Features
- Vision-guided red box sorting (HSV + OpenCV)
- MoveIt! motion planning (RRTConnect, 98% success)
- Gazebo simulation (177MB complete workspace)
- Docker: reproducible worldwide

## rights are reserved to the owner of the project
  Huang, L., Zhao, H., Implementation of UR5 pick and place in ROS-Gazebo with a USB cam and vacuum grippers, (2018), GitHub repository, https://github.com/lihuang3/ur5_ROS-Gazebo.git

I just have updated some of the outdated files, because the original repo was't updated since 8 years.

Much appreciation to the beautiful and fascinating work of both Huang, L., Zhao, H.


##  Quick Start

```bash
git clone https://github.com/ali76-AFK/UR5-Manipulator-with-Ubuntu-16.04-ROS1-kinetic-and-color-Recognition.git
cd UR5-Manipulator-with-Ubuntu-16.04-ROS1-kinetic-and-color-Recognition
docker build -t ur5_kinetic .
xhost +local:root
docker run -it --name ur5 --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ur5_kinetic bash

```

## Inside Containe

```bash
source ur5_kinetic_ws/devel/setup.bash
roslaunch ur5_notebook initialize.launch  # Gazebo + UR5 sim

```

## New terminal (docker exec -it ur5 bash):

```bash
source ur5_kinetic_ws/devel/setup.bash
rosrun ur5_notebook ur5_mp.py  # Autonomous pick-place!
```
## Videos and Screenshots Available in Media file
