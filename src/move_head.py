#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_head', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "head"
group = moveit_commander.MoveGroupCommander(group_name)
display_trajectory_publisher = rospy.Publisher(
    '/move_group/display_planned_path',
    moveit_msgs.msg.DisplayTrajectory,
     queue_size=10)

pose_target = geometry_msgs.msg.Pose()

#position:
pose_target.position.x = 0.072918715307
pose_target.position.y = -0.00803866779419
pose_target.position.z = 0.172490808626
#orientation:
pose_target.orientation.x = 0.0072681214306
pose_target.orientation.y = 0.132257600595
pose_target.orientation.z = -0.0543880014597
pose_target.orientation.w = 0.989695431329

group.set_pose_target(pose_target)

#group.set_random_target()

plan1 = group.plan()

rospy.sleep(5)

group.go(wait=True)

group.stop()
group.clear_pose_targets()

moveit_commander.roscpp_shutdown()
