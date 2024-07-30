# susan_apps
This package contains several applications related to Susan the 7-DOF robotic arm.

## 0. Prerequisites
* Ubuntu 20.04
* ROS Noetic

## 1. Dependencies
```sh
sudo apt-get install can-utils net-tools
```

## 2. Installation
```sh
mkdir -p susan_ws/src
cd susan_ws/src
git clone https://github.com/0nhc/susan_control.git
git clone https://github.com/0nhc/susan_description.git
git clone https://github.com/0nhc/susan_moveit.git
git clone https://github.com/0nhc/susan_apps.git
cd ..
rosdep install --from-path src --ignore-src -r -y
catkin_make
```

## 3. Examples

### 3.1 MoveIt Demo
The first terminal launches the MoveIt application.
```sh
# Terminal 1
cd susan_ws
source devel/setup.bash
roslaunch susan_moveit susan_moveit.launch
```

The second terminal launches the hardware interface. You can select hardware type between "fake" and "real".
```sh
# Terminal 2

# Setup your USB-CAN device firstly.
sudo ip link set can0 type can bitrate 1000000
sudo ifconfig can0 up

# Then start the hardware interface node.
cd susan_ws
source devel/setup.bash
roslaunch susan_control susan_control.launch hardware_type:=real

# You can also choose fake hardware for debugging:
# roslaunch susan_control susan_control.launch hardware_type:=fake
```
Then you can use MoveIt GUI to control the robotic arm.

## 3.2 Follow a Trajectory
To generate a trajectory, you can run [trajectory_generator.py](./scripts/trajectory_generator.py). Remember to specify an output folder in line 76 of [trajectory_generator.py](./scripts/trajectory_generator.py):
```python
# It indicates where the trajectory file is generated.
tg.save_traj("/home/hzx")
```
After you have specified a trajectory, you can make the robotic arm to follow the trajectory. To start the robotic arm, just execute all the steps in [3.1 MoveIt Demo](#31-moveit-demo):
```sh
# Terminal 1
cd susan_ws
source devel/setup.bash
roslaunch susan_moveit susan_moveit.launch

# Terminal 2
# It's recommended to use fake hardware for debugging firstly because real hardware may cause dangers in real world.
cd susan_ws
source devel/setup.bash
roslaunch susan_control susan_control.launch hardware_type:=fake
```
Then run [move_to_point.py](./scripts/move_to_point.py) to make the robotic arm following the trajectory generated. Note that you should firstly specity the path to the trajectory file in line 251 of [move_to_point.py](./scripts/move_to_point.py)：
```python
# It indicates the path to the trajectory file.
susan.follow_goal_trajectory("/home/hzx/trajectory.pickle")
```
Then you can start a new terminal to launch this node:
```sh
# Terminal 3
cd susan_ws
source devel/setup.bash
rosrun susan_apps move_to_point.py
```