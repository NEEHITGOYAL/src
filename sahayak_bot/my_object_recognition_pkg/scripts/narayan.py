#! /usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from tf import TransformListener
from object_msgs.msg import ObjectPose


#var_handle_pub = rospy.Publisher('/detection_info', ObjectPose, queue_size=10)

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('grasp_demo', anonymous=True)
robot = moveit_commander.RobotCommander()

arm_group = moveit_commander.MoveGroupCommander("arm_planning_group")
# Put the arm in the start position

hand_group = moveit_commander.MoveGroupCommander("grip_planning_group")
# Put the grip in the start position

arm_group.set_named_target("object4")
plan1 = arm_group.go()


t = TransformListener()

#t.waitForTransform("/ebot_base", "/object_102", rospy.Time(), rospy.Duration(4.0)) ###GLUE
#if t.frameExists("odom") and t.frameExists("object_101"):
#(glue,rotation) = t.lookupTransform("/ebot_base", "/object_102", rospy.Time())

t.waitForTransform("/ebot_base", "/object_99", rospy.Time(), rospy.Duration(4.0))
(coke,rotation) = t.lookupTransform("/ebot_base", "/object_99", rospy.Time())
###### GLUE

#var_handle_pub.publish('glue','translation')
#pose_target = geometry_msgs.msg.Pose()
#pose_target.orientation.w = 0.0
#pose_target.orientation.x = -0.0
#pose_target.orientation.y = 1.0
#pose_target.orientation.z = 0.0
#pose_target.position = translation 
#pose_target.position.y = glue[1]-0.245
#pose_target.position.z = glue[2]+0.05
#arm_group.set_pose_target(pose_target)
#arm_group.set_goal_tolerance(0.01)
#plan4 = arm_group.go()

#pose_target = geometry_msgs.msg.Pose()
#pose_target.orientation.w = 0.0
#pose_target.orientation.x = -0.0
#pose_target.orientation.y = 1.0
#pose_target.orientation.z = 0.0
#pose_target.position = translation 
#pose_target.position.x = glue[0]+0.008
#pose_target.position.y = glue[1]-0.235
#pose_target.position.z = glue[2]+0.008
#arm_group.set_pose_target(pose_target)
#arm_group.set_goal_tolerance(0.01)
#plan4 = arm_group.go()

#hand_group.set_named_target("close_glue")
#plan2 = hand_group.go()

#pose_target = geometry_msgs.msg.Pose()
#pose_target.orientation.w = 0.0
#pose_target.orientation.x = -0.0
#pose_target.orientation.y = 1.0
#pose_target.orientation.z = 0.0
#pose_target.position = translation 
#pose_target.position.x = glue[0]
#pose_target.position.y = glue[1]-0.220
#pose_target.position.z = glue[2]+0.2
#arm_group.set_pose_target(pose_target)
#arm_group.set_goal_tolerance(0.02)
#plan4 = arm_group.go()

#arm_group.set_named_target("frontup")
#plan1 = arm_group.go()

#arm_group.set_named_target("drop")
#plan1 = arm_group.go()

#hand_group.set_named_target("open")
#plan2 = hand_group.go()


###### COKE

pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 0.0
pose_target.orientation.x = -0.0
pose_target.orientation.y = 1.0
pose_target.orientation.z = 0.0
#pose_target.position = translation 
pose_target.position.x = coke[0]
pose_target.position.y = coke[1]-0.235
pose_target.position.z = coke[2]+0.07
arm_group.set_pose_target(pose_target)
arm_group.set_goal_tolerance(0.01)
plan4 = arm_group.go()

pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 0.0
pose_target.orientation.x = -0.0
pose_target.orientation.y = 1.0
pose_target.orientation.z = 0.0
#pose_target.position = translation 
pose_target.position.x = coke[0]+0.003
pose_target.position.y = coke[1]-0.215
pose_target.position.z = coke[2]+0.008
arm_group.set_pose_target(pose_target)
arm_group.set_goal_tolerance(0.01)
plan4 = arm_group.go()

#hand_group.set_named_target("close_coke")
#plan2 = hand_group.go()

#pose_target = geometry_msgs.msg.Pose()
#pose_target.orientation.w = 0.0
#pose_target.orientation.x = -0.0
#pose_target.orientation.y = 1.0
#pose_target.orientation.z = 0.0
#pose_target.position = translation 
#pose_target.position.x = coke[0]
#pose_target.position.y = coke[1]-0.235
#pose_target.position.z = coke[2]+0.2
#arm_group.set_pose_target(pose_target)
#arm_group.set_goal_tolerance(0.02)
#plan4 = arm_group.go()

#arm_group.set_named_target("frontup")
#plan1 = arm_group.go()

#arm_group.set_named_target("drop")
#plan1 = arm_group.go()

#hand_group.set_named_target("open")
#plan2 = hand_group.go()

#arm_group.set_named_target("Straightup")
#plan1 = arm_group.go()

rospy.sleep(5)
moveit_commander.roscpp_shutdown()
