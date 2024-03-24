# Project 2: Go Chase It

Create a robot using ROS that follows a white ball.


## Structure
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


## Build and Launch

Load Protocol:
mkdir /home/workspace/catkin_ws/
cd /home/workspace/catkin_ws/
git clone https://github.com/rsacred/Project2 src


	//Init the catkin workspace

cd src
catkin_init_workspace

	//Build the catkin package

cd /home/workspace/catkin_ws
catkin_make

	//Launch the gazebo simulation On a first terminal run

source devel/setup.bash
roslaunch my_robot world.launch

	//Launch drive_bot and process_image nodes Open a new terminal and launch the ball_chaser:

cd /home/workspace/catkin_ws
source devel/setup.bash
roslaunch ball_chaser ball_chaser.launch

Now you can move the white ball in the world so that the robot will follow it.