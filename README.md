**db_demo**

Simple package containing a basic commander for UR5.

**prerequisite**
For these tests Universal_robot package was installed from source.
Check repository.rosinstall
**Robot launch**
To launch the robot use 

'''bash
roslaunch ur_bringup ur5_bringup.launch robot_ip:=<robot ip>
'''

robot ip should be set accordingly. PC and robot should be on the same network.

To launch moveit_config files use
'''bash
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch limited:=true
'''

To launch rviz
'''bash
rosrun rviz rviz
'''

To launch the warehouse use
'''bash
roslaunch sr_multi_moveit_config default_warehouse_db.launch
'''
