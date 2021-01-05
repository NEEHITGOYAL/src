#! /usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from tf import TransformListener

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('grasp_demo', anonymous=True)
robot = moveit_commander.RobotCommander()

arm_group = moveit_commander.MoveGroupCommander("arm_planning_group")
# Put the arm in the start position

hand_group = moveit_commander.MoveGroupCommander("grip_planning_group")
# Put the grip in the start position

arm_group.set_named_target("object6")
plan1 = arm_group.go()


t = TransformListener()

t.waitForTransform("/odom", "/object_102", rospy.Time(), rospy.Duration(4.0))
#if t.frameExists("odom") and t.frameExists("object_101"):
(translation,rotation) = t.lookupTransform("/odom", "/object_102", rospy.Time())


pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 0.0
pose_target.orientation.x = 0.0
pose_target.orientation.y = 0.0
pose_target.orientation.z = 0.0
pose_target.position = translation 
pose_target.position.z = 1.0
arm_group.set_pose_target(pose_target)
plan4 = arm_group.go()





#arm_group.set_named_target("Straightup")
#plan1 = arm_group.go()

rospy.sleep(5)
moveit_commander.roscpp_shutdown()
