# ROS Noetic Project Setup

## Prerequisites
Before running the project, ensure you have the following installed on your system:

1. ROS Noetic: Follow the official ROS Noetic installation instructions for Ubuntu 20.04 [here](http://wiki.ros.org/noetic/Installation/Ubuntu).

2. Install the required ROS packages:

```bash
  sudo apt install ros-noetic-slam-gmapping ros-noetic-robot-controllers ros-noetic-rgbd-launch ros-noetic-tf2-sensor-msgs ros-noetic-moveit-core ros-noetic-move-base-msgs
```

3. Install the following additional dependencies 

```bash
  sudo apt-get install cmake libblkid-dev e2fslibs-dev libboost-all-dev libaudit-dev build-essential g++ python-dev autotools-dev libicu-dev libbz2-dev libsdl1.2-dev libsdl-image1.2-dev terminator
```

4. Create a directory named "include" in your root folder

```bash
  sudo mkdir include
```

5. Create a Conda environment named "fetch" with Python 3.8 and install the following

```bash
conda create --name fetch python=3.8
conda activate fetch
conda install numpy
conda install defusedxml
conda install rospkg
```

6. To run the project, use the following commands in separate terminal windows:
   
```bash
   terminator
```

7. In the first Terminator window, launch the Gazebo simulation:

```bash
  roslaunch fetch_gazebo simulation.launch
```

8. In the second Terminator window, launch the navigation:

```bash
  roslaunch fetch_gazebo nav.launch
```

9. In the third Terminator window, launch RViz with a custom configuration:

```bash
  rosrun rviz rviz -d myrviz.rviz
```

10. If you get Could NOT find SDL (missing: SDL_LIBRARY SDL_INCLUDE_DOR) error
11. 
```bash
  sudo apt-get install libsdl1.2-dev
  sudo apt-get install libsdl-image1.2-dev
```


