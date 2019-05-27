#!/usr/bin/env python
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

class initial_position(object):
  """initial_position"""
  def __init__(self):
    super(initial_position, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name1 = "base"
    group_name2 = "head"
    group_name3 = "both_arms"
    group_name4 = "both_legs"

    move_group1 = moveit_commander.MoveGroupCommander(group_name1)
    move_group2 = moveit_commander.MoveGroupCommander(group_name2)
    move_group3 = moveit_commander.MoveGroupCommander(group_name3)
    move_group4 = moveit_commander.MoveGroupCommander(group_name4)


    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

    self.robot = robot
    self.move_group1 = move_group1
    self.move_group2 = move_group2
    self.move_group3 = move_group3
    self.move_group4 = move_group4
    self.scene = scene


  def go_to_joint_state(self):
      robot=self.robot
      move_group1 = self.move_group1
      move_group2 = self.move_group2
      move_group3 = self.move_group3
      move_group4 = self.move_group4
      scene = self.scene

      #BOTH_LEGS
      joint_goal3 = move_group4.get_current_joint_values()
      joint_goal3[0] = 0
      joint_goal3[1] = 0
      joint_goal3[2] = 0
      joint_goal3[3] = 0
      joint_goal3[4] = 0
      joint_goal3[5] = 0
      joint_goal3[6] = 0
      joint_goal3[7] = 0
      joint_goal3[8] = 0
      joint_goal3[9] = 0
      joint_goal3[10] = 0
      joint_goal3[11] = 0
      move_group4.go(joint_goal3, wait=True)

      #BOTH_ARMS
      joint_goal2 = move_group3.get_current_joint_values()
      joint_goal2[0] = 0
      joint_goal2[1] = 0
      joint_goal2[2] = 0
      joint_goal2[3] = -0.8
      joint_goal2[4] = 0
      joint_goal2[5] = 0
      joint_goal2[6] = 0
      joint_goal2[7] = 0
      joint_goal2[8] = 0
      joint_goal2[9] = 0.8
      joint_goal2[10] = 0
      joint_goal2[11] = 0
      move_group3.go(joint_goal2, wait=True)

      #BASE
      joint_goal = move_group1.get_current_joint_values()
      joint_goal[0] = 0
      joint_goal[1] = 0
      joint_goal[2] = 0
      move_group1.go(joint_goal, wait=True)

      #HEAD
      joint_goal1 = move_group2.get_current_joint_values()
      joint_goal1[0] = 0
      joint_goal1[1] = 0
      move_group2.go(joint_goal1, wait=True)

      rospy.sleep(1)

      ## Getting Basic Information
      print "Printing robot state"
      print robot.get_current_state()
      print ""

def main():
  try:
    print ""
    print "-------------------------------------------------"
    print "Control articular del NAO: posición inicial"
    print "-------------------------------------------------"

    tutorial = initial_position()
    tutorial.go_to_joint_state()

    print "¡Control completado con éxito!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

#JOINT VALUES FOR INITIAL position
#OBTAINED EXCUTING --> rosrun nao_get_data get_data.py
# Current Joint Values of group:base
# [0.0, 0.0, 0.0]
#
# Current Joint Values of group:head
# [0.0, 0.0]
#
# Current Joint Values of group:both_arms
# [0.0, 0.0, 0.0, -0.7897633000000001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.7897633000000001, 0.0, 0.0]
#
# Current Joint Values of group:both_legs
# [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
