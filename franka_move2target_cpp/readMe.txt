Task:
    1. Create a publisher node to pulish a target pose to franka arm
    2. Create a subscriber node to listen each joint value
    3. Create a controller node to move the arm from current pose to the target pose
    4. Create a launch file to view the move in RViz

Tools:
    1. Ubuntu 22.04
    2. Ros2 humble
    3. moveit2

Workspace:
    Add the package under the src of moveit2 workspace 
    If you create the moveit2 workapce by following the moveit2 humble tutorial 
    The direction should be: ~/ws_moveit2/src

Step 1: Build the package
    colcon build --packages-select franka_move2target_cpp

Step 2: Open the first Terminal: 
        source install/setup.bash
        ros2 launch franka_move2target_cpp rviz.launch.py 

Step 3: Open the second Terminal
        source install/setup.bash
        ros2 run franka_move2target_cpp franka_targetPose_publisher 

Step 4: Open the thired Terminal
        source install/setup.bash
        ros2 run franka_move2target_cpp franka_jointState_listener 

Step 5: Open the forth Terminal
        source install/setup.bash
        ros2 run franka_move2target_cpp franka_controller_cpp 

Result: 
    You should see the move of the arm in RViz,
    The angle value of each joint should be printed in the thired Terminal 
