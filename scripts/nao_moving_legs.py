#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import time

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *

def all_close(goal, actual, tolerance):
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class NaoMovingLegs(object):
  """NaoMovingLegs"""
  def __init__(self):
    super(NaoMovingLegs, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('nao_grasping', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name0 = "base"
    group_name1 = "right_arm"
    group_name2 = "left_arm"
    group_name3 = "right_leg"
    group_name4 = "left_leg"
    group_name5 = "both_arms"
    group_name6 = "both_legs"
    move_group0 = moveit_commander.MoveGroupCommander(group_name0)
    move_group1 = moveit_commander.MoveGroupCommander(group_name1)
    move_group2 = moveit_commander.MoveGroupCommander(group_name2)
    move_group3 = moveit_commander.MoveGroupCommander(group_name3)
    move_group4 = moveit_commander.MoveGroupCommander(group_name4)
    move_group5 = moveit_commander.MoveGroupCommander(group_name5)
    move_group6 = moveit_commander.MoveGroupCommander(group_name6)

    display_trajectory_publisher = rospy.Publisher(
    '/move_group/display_planned_path',
    moveit_msgs.msg.DisplayTrajectory, queue_size=10)

    ## Getting Basic Information
    planning_frame = move_group1.get_planning_frame()
    print "Planning frame: %s" % planning_frame
    eef_link_r = move_group1.get_end_effector_link()
    print "End effector link for right arm: %s" % eef_link_r
    eef_link_l = move_group2.get_end_effector_link()
    print "End effector link for left arm: %s" % eef_link_l
    group_names = robot.get_group_names()
    print "Available Planning Groups:", robot.get_group_names()
    print "Printing robot state"
    print robot.get_current_state()
    print ""

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group0 = move_group0
    self.move_group1 = move_group1
    self.move_group2 = move_group2
    self.move_group3 = move_group3
    self.move_group4 = move_group4
    self.move_group5 = move_group5
    self.move_group6 = move_group6
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link_r = eef_link_r
    self.eef_link_l = eef_link_l
    self.group_names = group_names
    self.group_name1 = group_name1

  def nao_moving_legs_func(self):
    move_group0 = self.move_group0
    move_group1 = self.move_group1
    move_group2 = self.move_group2
    move_group3 = self.move_group3
    move_group4 = self.move_group4
    move_group5 = self.move_group5
    move_group6 = self.move_group6
    scene = self.scene
    box_name = self.box_name
    robot = self.robot
    eef_link_r = self.eef_link_r
    eef_link_l = self.eef_link_l
    group_names = self.group_names
    group_name1 = self.group_name1

    ############################ MOVING BASE #############
    ######articular control ########
    joint_goal = move_group0.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = 0.2
    move_group0.go(joint_goal, wait=True)

    ############################ MOVING LEGS #############
    ######articular control ########
    joint_goal = move_group4.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = 0.2
    # joint_goal[2] = -0.6 #-1.4
    # joint_goal[3] = 1.7 #2.1
    # joint_goal[4] = -1.1 #-1.1
    # joint_goal[5] = 0 #0
    move_group4.go(joint_goal, wait=True)


def main():
  try:
    print ""
    print "-------------------------------------------------"
    print "Joint control with NAO: moving legs"
    print "-------------------------------------------------"
    print ""

    tutorial = NaoMovingLegs()
    tutorial.nao_moving_legs_func()

    print "Control succedssfully completed!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
