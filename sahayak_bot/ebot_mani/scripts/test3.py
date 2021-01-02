#! /usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('test3', anonymous=True)
robot = moveit_commander.RobotCommander()
hand_group = moveit_commander.MoveGroupCommander("gripper_planning_group")
arm_group = moveit_commander.MoveGroupCommander("ur5_1_planning_group")
# Put the arm in the start position
arm_group.set_named_target("Straightup")
plan1 = arm_group.go()

hand_group.set_named_target("Open")
plan2 = hand_group.go()

pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 0.261
pose_target.orientation.x = -0.261
pose_target.orientation.y = 0.657
pose_target.orientation.z = -0.657
pose_target.position.x = 0.460000
pose_target.position.y = 0.220000
pose_target.position.z = 0.922496
arm_group.set_pose_target(pose_target)
plan4 = arm_group.go()


pose_target.position.z = 0.832496
arm_group.set_pose_target(pose_target)
plan4 = arm_group.go()


# Put the grip in the start position
hand_group.set_named_target("Close")
plan2 = hand_group.go()

pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 0.5
pose_target.orientation.x = -0.5
pose_target.orientation.y = 0.5
pose_target.orientation.z = -0.5
pose_target.position.x = 0.000000
pose_target.position.y = -0.710000
pose_target.position.z = 1.105000
arm_group.set_pose_target(pose_target)
plan4 = arm_group.go()

hand_group.set_named_target("Open")
plan3 = hand_group.go()

pose_target.orientation.w = 0.707
pose_target.orientation.x = -0.707
pose_target.orientation.y = 0.0
pose_target.orientation.z = 0.0
pose_target.position.x = 0.540000
pose_target.position.y = -0.240000
pose_target.position.z = 0.942496
arm_group.set_pose_target(pose_target)
plan4 = arm_group.go()

pose_target.position.z = 0.88
arm_group.set_pose_target(pose_target)
plan4 = arm_group.go()

# Put the grip in the start position
hand_group.set_named_target("Close2")
plan2 = hand_group.go()

pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 0.5
pose_target.orientation.x = -0.5
pose_target.orientation.y = 0.5
pose_target.orientation.z = -0.5
pose_target.position.x = 0.000000
pose_target.position.y = -0.710000
pose_target.position.z = 1.105000
arm_group.set_pose_target(pose_target)
plan4 = arm_group.go()

hand_group.set_named_target("Open")
plan3 = hand_group.go()

pose_target.orientation.w = 0.653
pose_target.orientation.x = -0.653
pose_target.orientation.y = 0.272
pose_target.orientation.z = -0.272
pose_target.position.x = 0.560000
pose_target.position.y = 0.00000
pose_target.position.z = 0.942496
arm_group.set_pose_target(pose_target)
plan4 = arm_group.go()

pose_target.position.z = 0.862496
arm_group.set_pose_target(pose_target)
plan4 = arm_group.go()

# Put the grip in the start position
hand_group.set_named_target("Close3")
plan2 = hand_group.go()

pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 0.5
pose_target.orientation.x = -0.5
pose_target.orientation.y = 0.5
pose_target.orientation.z = -0.5
pose_target.position.x = 0.000000
pose_target.position.y = 0.710000
pose_target.position.z = 1.105000
arm_group.set_pose_target(pose_target)
plan4 = arm_group.go()

hand_group.set_named_target("Open")
plan3 = hand_group.go()

rospy.sleep(2)
moveit_commander.roscpp_shutdown()
