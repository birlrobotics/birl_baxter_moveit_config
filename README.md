#  birl baxter moveit config 
## Summary
This repo introduces how to use Birl Baxter Research Robot with MoveIt! The standard ROS motion planing framework

##  Installation/Prerequisites
Install the packages below,
[birl_baxter_common](https://github.com/birlrobotics/birl_baxter_common.git)
[moveit_robots](https://github.com/ros-planning/moveit_robots.git)
 [Read this web](http://sdk.rethinkrobotics.com/wiki/MoveIt_Tutorial)
 [Baxter simulation](http://sdk.rethinkrobotics.com/wiki/Baxter_Simulator)


##  move_group
    <group name="right_arm">
       <chain base_link="torso" tip_link="right_gripper" />
    </group>
    <group name="left_arm">
        <chain base_link="torso" tip_link="left_gripper" />
    </group>
    <group name="right_hand">
        <link name="right_hand" />
        <link name="FT_bottom_plate_right" />
        <link name="FT_right" />
        <link name="FT_Top_Plate_right" />
        <link name="right_gripper_base" />
        <link name="r_gripper_l_finger" />
        <link name="r_gripper_l_finger_tip" />
        <link name="r_gripper_r_finger" />
        <link name="r_gripper_r_finger_tip" />
        <link name="right_gripper" />
    </group>
    <group name="left_hand">
        <link name="left_hand" />
        <link name="FT_bottom_plate_left" />
        <link name="FT_left" />
        <link name="FT_Top_Plate_left" />
        <link name="left_gripper_base" />
        <link name="l_gripper_l_finger" />
        <link name="l_gripper_l_finger_tip" />
        <link name="l_gripper_r_finger" />
        <link name="l_gripper_r_finger_tip" />
        <link name="left_gripper" />
    </group>
    <group name="both_arm">
        <group name="right_arm" />
        <group name="left_arm" />
    </group>


## Tutorial
### Dummy Robot
Just launch this command

    roslaunch birl_baxter_moveit_config demo.launch
### Connect to simulation robot
Start Baxter simulation

    roslaunch baxter_gazebo baxter_world.launch


Verify that the robot is enabled from an RSDK terminal session, ex: 

    $ rosrun baxter_tools enable_robot.py -e
Start the joint trajectory controller, ex: 
 

    $ rosrun baxter_interface joint_trajectory_action_server.py
 Remapping `/robot/joint_states` to  `joint_states`

    $ rosrun birl_baxter_moveit_config joint_states_remap_topic.py
   
   Start MoveIt
   
    roslaunch birl_baxter_moveit_config birl_baxter_gripper.launch

   
### Connect to real robot 
Repeat all the command above except start Baxter simulation instead of getting started with

    ./baxter.sh

