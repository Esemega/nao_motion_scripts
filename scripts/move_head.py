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

class move_head(object):
  """move_head"""
  def __init__(self):
    super(move_head, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name1 = "head"

    move_group1 = moveit_commander.MoveGroupCommander(group_name1)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

    self.robot = robot
    self.move_group1 = move_group1

  def go_to_joint_state(self):
      robot=self.robot
      move_group1 = self.move_group1
      #HEAD
      joint_goal = move_group1.get_current_joint_values()
      joint_goal[0] = 0
      joint_goal[1] = 0.2
      move_group1.go(joint_goal, wait=True)

      rospy.sleep(1)

def main():
  try:
    print ""
    print "-------------------------------------------------"
    print "Control articular del NAO: inclinar cabeza"
    print "-------------------------------------------------"

    tutorial = move_head()
    tutorial.go_to_joint_state()

    print "¡Control completado con éxito!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

#JOINT VALUES FOR CURRENT POSITION
#OBTAINED EXCUTING --> rosrun nao_get_data get_data.py
# Current Joint Values of group:head
# [-1.1927357835102732e-05, 0.21270936022381687]
