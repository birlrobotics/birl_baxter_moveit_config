#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley

## BEGIN_SUB_TUTORIAL imports
##
## To use the python interface to move_group, import the moveit_commander
## module.  We also import rospy and some messages that we will use.
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import ipdb
## END_SUB_TUTORIAL

from std_msgs.msg import String

def move_group_python_interface_tutorial():
  ## BEGIN_TUTORIAL
  ##
  ## Setup
  ## ^^^^^
  ## CALL_SUB_TUTORIAL imports
  ##
  ## First initialize moveit_commander and rospy.
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)

  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
  robot = moveit_commander.RobotCommander()

  ## Instantiate a PlanningSceneInterface object.  This object is an interface
  ## to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()

  ## Instantiate a MoveGroupCommander object.  This object is an interface
  ## to one group of joints.  In this case the group is the joints in the left
  ## arm.  This interface can be used to plan and execute motions on the left
  ## arm.
  group = moveit_commander.MoveGroupCommander("right_arm")
  


  ## We create this DisplayTrajectory publisher which is used below to publish
  ## trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory)

  ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
  print "============ Waiting for RVIZ..."
  rospy.sleep(1)
  print "============ Starting tutorial "

  ## Getting Basic Information
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^
  ##
  ## We can get the name of the reference frame for this robot
  print "============ Reference frame: %s" % group.get_planning_frame()

  ## We can also print the name of the end-effector link for this group
  print "============ Reference frame: %s" % group.get_end_effector_link()

  ## We can get a list of all the groups in the robot
  print "============ Robot Groups:"
  print robot.get_group_names()

  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot.
  print "============ Printing robot state"
  print robot.get_current_state()
  print "============"

  # Set the reference frame for pose targets
  reference_frame = 'base'
  # Set the right arm reference frame accordingly
  group.set_pose_reference_frame(reference_frame)
  
  # Allow replanning to increase the odds of a solution
  group.allow_replanning(True)
  
  # Allow some leeway in position (meters) and orientation (radians)
  group.set_goal_position_tolerance(0.05)
  group.set_goal_orientation_tolerance(0.1)
  
  # Get the name of the end-effector link
  end_effector_link = group.get_end_effector_link()

  ## Planning to a Pose goal
  ## ^^^^^^^^^^^^^^^^^^^^^^^
  ## We can plan a motion for this group to a desired pose for the 
  ## end-effector
  print "============ Generating plan 1"
  pose_target = geometry_msgs.msg.Pose()
  pose_target.orientation.w = 0.20149958079 
  pose_target.orientation.x = 0.0701981898529
  pose_target.orientation.y = -0.66496014039
  pose_target.orientation.z = -0.715750057475
  pose_target.position.x = 0.380451105904
  pose_target.position.y = -0.301373844018
  pose_target.position.z = 0.68235969972
  
  # Set the start state to the current state
  group.set_start_state_to_current_state()
  # Set the goal pose of the end effector to the stored pose
  group.set_pose_target(pose_target, end_effector_link)
  
  # Plan the trajectory to the goal
  traj = group.plan()
  
  # Execute the planned trajectory
  group.execute(traj)
  rospy.sleep(1)

  ## Cartesian Paths
  ## ^^^^^^^^^^^^^^^
  ## You can plan a cartesian path directly by specifying a list of waypoints 
  ## for the end-effector to go through.
  waypoints = []
  wpose1 = geometry_msgs.msg.Pose()
  wpose1.position.x = 0.607157402657 
  wpose1.position.y = -0.172825390916
  wpose1.position.z = 0.2592603354757
  wpose1.orientation.w = 0.0171719685224
  wpose1.orientation.x = -0.99309332868
  wpose1.orientation.y = -0.114493381304
  wpose1.orientation.z = -0.0190270771749
  waypoints.append(wpose1)

  wpose2 = geometry_msgs.msg.Pose()
  wpose2.position.x = 0.607157402657 
  wpose2.position.y = -0.172825390916
  wpose2.position.z = 0.2692603354757
  wpose2.orientation.w = 0.0171719685224
  wpose2.orientation.x = -0.99309332868
  wpose2.orientation.y = -0.114493381304
  wpose2.orientation.z = -0.0190270771749
  waypoints.append(wpose2)

  wpose3 = geometry_msgs.msg.Pose()
  wpose3.position.x = 0.607157402657 
  wpose3.position.y = -0.172825390916
  wpose3.position.z = 0.282603354757
  wpose3.orientation.w = 0.0171719685224
  wpose3.orientation.x = -0.99309332868
  wpose3.orientation.y = -0.114493381304
  wpose3.orientation.z = -0.0190270771749
  waypoints.append(wpose3)
  ## We want the cartesian path to be interpolated at a resolution of 1 cm
  ## which is why we will specify 0.01 as the eef_step in cartesian
  ## translation.  We will specify the jump threshold as 0.0, effectively
  ## disabling it.
  (plan3, fraction) = group.compute_cartesian_path(
                               waypoints,   # waypoints to follow
                               0.01,        # eef_step
                               0.0)         # jump_threshold
                               
  print "============ Waiting while RVIZ displays plan3..."
  print "============ Visualizing plan1"
  
  display_trajectory = moveit_msgs.msg.DisplayTrajectory()
  display_trajectory.trajectory_start = robot.get_current_state()
  display_trajectory.trajectory.append(plan3)
  display_trajectory_publisher.publish(display_trajectory);
  rospy.sleep(4)
  print "============ Waiting while plan3 is visualized (again)..."
  group.execute(plan3)
#  
#  
#  
#
# 
#  ## Adding/Removing Objects and Attaching/Detaching Objects
#  ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#  ## First, we will define the collision object message
#  collision_object = moveit_msgs.msg.CollisionObject()



  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()

  ## END_TUTORIAL

  print "============ STOPPING"


if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass

