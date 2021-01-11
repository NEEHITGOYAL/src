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

arm_group.set_named_target("photo")
plan1 = arm_group.go()
hand_group.set_named_target("open")
plan1 = hand_group.go()

t = TransformListener()

t.waitForTransform("/ebot_base", "/object_110", rospy.Time(), rospy.Duration(4.0))
#if t.frameExists("odom") and t.frameExists("object_101"):
(translation,rotation) = t.lookupTransform("/ebot_base", "/object_110", rospy.Time())

#var_handle_pub.publish('glue','translation')
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 0.0
pose_target.orientation.x = 0.0
pose_target.orientation.y = 1.0
pose_target.orientation.z = 0.0
#pose_target.position = translation 
pose_target.position.x = translation[0] -0.02
pose_target.position.y = translation[1]-0.350
pose_target.position.z = translation[2]+0.02
print("x={},y={},z={}".format(translation[0],translation[1],translation[2]))
arm_group.set_pose_target(pose_target)
               
plan4 = arm_group.go()

pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 0.0
pose_target.orientation.x = 0.0
pose_target.orientation.y = 1.0
pose_target.orientation.z = 0.0
#pose_target.position = translation 
pose_target.position.x = translation[0] -0.02
pose_target.position.y = translation[1] - 0.19
pose_target.position.z = translation[2] 
print("x={},y={},z={}".format(translation[0],translation[1],translation[2]))
arm_group.set_pose_target(pose_target)
               
plan4 = arm_group.go()

hand_group.set_named_target("close1")
plan1 = hand_group.go()



#arm_group.set_named_target("Straightup")
#plan1 = arm_group.go()

rospy.sleep(5)
moveit_commander.roscpp_shutdown()
