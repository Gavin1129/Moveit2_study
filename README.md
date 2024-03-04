# Moveit2_study
## moveit2_example
movit2 simple example code from Moveit2 tutorial 
## franka_move2target
example code to control franka research 3 robot to desired pose in rviz2
## Franka_Sigma_ws
Example code to control franka research 3 robot through haptic device sigma.7 

## Move the Franka hardware to desired pose
Step 1:
``ros2 launch franka_moveit_config franka_moveit.launch.py robot_ip:=172.16.0.2 use_fake_hardware:=false``
It will turn on the Rviz2, go to "Panels->add new panel->RvizVisualToolsGUI->ok" to add visual tool

Step 2: 
``ros2 run franka_moveit_my hello_moveit``


