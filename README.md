# Project 2: Go Chase It!

This project involves the creation of a ROS package that enables a robot to follow a white ball.


## File Structure
```
    .Project2                          # Go Chase It Project
    ├── my_robot                       # my_robot package                   
    │   ├── launch                     # launch folder for launch files   
    │   │   ├── robot_description.launch
    │   │   ├── world.launch
    │   ├── meshes                     # meshes folder for sensors
    │   │   ├── hokuyo.dae
    │   ├── urdf                       # urdf folder for xarco files
    │   │   ├── my_robot.gazebo
    │   │   ├── my_robot.xacro
    │   ├── world                      # world folder for world files
    │   │   ├── ryan.world
    │   ├── CMakeLists.txt             # compiler instructions
    │   ├── package.xml                # package info
    ├── ball_chaser                    # ball_chaser package                   
    │   ├── launch                     # launch folder for launch files   
    │   │   ├── ball_chaser.launch
    │   ├── src                        # source folder for C++ scripts
    │   │   ├── drive_bot.cpp
    │   │   ├── process_images.cpp
    │   ├── srv                        # service folder for ROS services
    │   │   ├── DriveToTarget.srv
    │   ├── CMakeLists.txt             # compiler instructions
    │   ├── package.xml                # package info                  
    └──   
```
## Launch Procedure

Load the project:
```
mkdir /home/workspace/catkin_ws/
cd /home/workspace/catkin_ws/
git clone https://github.com/rsacred/Project2 src
```

Initialize the catkin workspace:
```
cd src
catkin_init_workspace
```

Build the catkin package:
```
cd /home/workspace/catkin_ws
catkin_make
```

Launch the Gazebo simulation:
```
source devel/setup.bash
roslaunch my_robot world.launch
```

In a new terminal, launch the ball_chaser package by launching the drive_bot and process_image nodes:
```
cd /home/workspace/catkin_ws
source devel/setup.bash
roslaunch ball_chaser ball_chaser.launch
```

Now you can move the white ball in the world and observe the robot following it.
