#!/usr/bin/env python
import sys
import moveit_commander
import rospy

rospy.init_node('moveit_commander_test', anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)

print("MoveIt! Commander está instalado y funcionando correctamente.")
