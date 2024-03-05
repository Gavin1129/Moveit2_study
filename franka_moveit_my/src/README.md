Example:

Franka hardware subscribe target pose that published by another node and follow the target.

Steps:

1. Launch franka hardware
   ```
   ros2 launch franka_moveit_config franka_moveit.launch.py robot_ip:=172.16.0.2 use_fake_hardware:=false
   ```
2. Listener
   ```
   ros2 run franka_moveit_my follow_target 
   ```
3. Pubisher
   ```
   ros2 run franka_moveit_my franka_target_pub 
   ```
