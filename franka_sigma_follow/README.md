# Franka Hardware Follow Sigma7 Haptic Device

This is an example to control FR3 arm hardware by sigma.7 haptic device .

## Steps:

1. Start sigma7 device
   ```
   sudo su
   ros2 run sigma7 sigma_main
   ```
2. Launch Franka hardware configure
   ```
   ros2 launch franka_moveit_config franka_moveit.launch.py robot_ip:=172.16.0.2 use_fake_hardware:=false
   ```
3. Listen to Sigma7's pose
   ```
   ros2 run franka_sigma_follow franka_target_publisher 

   ```
4. Run follow command
   ```
   ros2 run franka_sigma_follow franka_follow_sigma
   ```

## Results

You will see the Franka arm follow the motion of your sigma7
