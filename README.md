pinky_behaviortree
==============

## 1. ROS2 pkg clone
```
mkdir -p ~/pinky_bt/src
cd ~/pinky_bt/src
git clone https://github.com/pinklab-art/pinky_behaviortree.git

```
## 2. ROS2 pkg build
```
cd ~/pinky_bt
rosdep install --from-paths src --ignore-src -r -y
colcon build
```
## 3. Pinky brinup
-------------
```
source ~/pinky_bt/install/local_setup.bash
ros2 launch pinky_gazebo office_world.launch.py 
```
## 4. SLAM
-------------
#### launch gmapping
```
ros2 launch pinky_slam gmapping_launch.py 
```
#### robot control
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```
#### map save 
```
ros2 run nav2_map_server map_saver_cli -f <map name>
```

### 5. Navigation2 
-------------
#### launch nav2
```
ros2 launch pinky_navigation navigation2.launch.py use_sim_time:=true
map:=<map name>
```




