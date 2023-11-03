# moveit_issue
This repository contains the code to reproduce the issues we have with moveit.

To reproduce the error you have to follow these steps (probably only works with linux and nvidia gpu):
1. Open this directory in vscode and reopen it in docker container. It will take quiet a while to build the docker.
2. Run `colcon build`
3. Open three terminals and run:
   1. `source install/setup.bash; ros2 launch moveit_door_opening_mechanism_rotating_arm_config moveit_rviz_simulation_launch.py`
   2. `source install/setup.bash; ros2 launch door_opening_mechanism_mtc door_opening_mechanism_mtc_launch.py `
   3. `source install/setup.bash; ros2 run door_opening_mechanism_mtc door_opening_mechanism_send_fake_pose`
4. Now you should receive these logs in the second terminal:
   
[door_opening_mechanism_mtc-1] terminate called after throwing an instance of 'std::runtime_error'
[door_opening_mechanism_mtc-1]   what():  Planning plugin name is empty. Please choose one of the available plugins: chomp_interface/CHOMPPlanner, ompl_interface/OMPLPlanner, pilz_industrial_motion_planner/CommandPlanner, stomp_moveit/StompPlanner
[ERROR] [door_opening_mechanism_mtc-1]: process has died [pid 20371, exit code -6, cmd '/workspace/install/door_opening_mechanism_mtc/lib/door_opening_mechanism_mtc/door_opening_mechanism_mtc --ros-args -r __node:=door_opening_mechanism_mtc --params-file /tmp/launch_params_at9cklmr --params-file /tmp/launch_params_iflmk3za --params-file /tmp/launch_params_62__pv0s'].

Any hint on what the issue could be would be highly appreciated! 