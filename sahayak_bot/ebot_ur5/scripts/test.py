#! /usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('test', anonymous=True)
robot = moveit_commander.RobotCommander()
hand_group = moveit_commander.MoveGroupCommander("grip_planning_group")
arm_group = moveit_commander.MoveGroupCommander("arm_planning_group")
# Put the arm in the start position
arm_group.set_named_target("frontup")
plan1 = arm_group.go()

#hand_group.set_named_target("Open")
#plan2 = hand_group.go()

pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 0
pose_target.orientation.x = 0
pose_target.orientation.y = 1
pose_target.orientation.z = 0
pose_target.position.x = 0.6
pose_target.position.y = 0.5
pose_target.position.z = 0.4
arm_group.set_pose_target(pose_target)
plan4 = arm_group.go()



rospy.sleep(2)
moveit_commander.roscpp_shutdown()
