#! /usr/bin/env python
import sys
import os
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
plan1=False
while(plan1==False):
    arm_group.set_named_target("photo")
    plan1 = arm_group.go()
plan1=False
while(plan1==False):    
    hand_group.set_named_target("open")
    plan1 = hand_group.go()

t = TransformListener()


t.waitForTransform("/ebot_base", "/object_110", rospy.Time(), rospy.Duration(4.0))
(coke,rotation1) = t.lookupTransform("/ebot_base", "/object_110", rospy.Time())
try:
    t.waitForTransform("/ebot_base", "/object_109", rospy.Time(), rospy.Duration(4.0))
    (battery,rotation2) = t.lookupTransform("/ebot_base", "/object_109", rospy.Time())
except:
    t.waitForTransform("/ebot_base", "/object_108", rospy.Time(), rospy.Duration(4.0))
    (battery,rotation2) = t.lookupTransform("/ebot_base", "/object_108", rospy.Time())   
t.waitForTransform("/ebot_base", "/object_106", rospy.Time(), rospy.Duration(4.0))
#if t.frameExists("odom") and t.frameExists("object_101"):
(glue,rotation3) = t.lookupTransform("/ebot_base", "/object_106", rospy.Time())
os.system("rosnode kill "+ '/find_object_3d')
os.system("rosnode kill "+ '/tf_example')
plan4=False
#var_handle_pub.publish('glue','translation')
while(plan4==False):
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = 0.0
    pose_target.orientation.x = 0.0
    pose_target.orientation.y = -0.972
    pose_target.orientation.z = 0.234
    #pose_target.position = translation 
    pose_target.position.x = coke[0] 
    pose_target.position.y = coke[1] -0.4
    pose_target.position.z = coke[2] +0.1
    arm_group.set_pose_target(pose_target)
    arm_group.set_goal_tolerance(0.01)              
    plan4 = arm_group.go()
    print("x={},y={},z={},plan1={}".format(coke[0],coke[1],coke[2],plan4))
plan4=False
while(plan4==False):  
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = 0.0
    pose_target.orientation.x = 0.0
    pose_target.orientation.y = -0.972
    pose_target.orientation.z = 0.234
    #pose_target.position = translation 
    pose_target.position.x = coke[0] 
    pose_target.position.y = coke[1] -0.1836
    pose_target.position.z = coke[2] +0.1
    arm_group.set_goal_tolerance(0.01)  
    arm_group.set_pose_target(pose_target)              
    plan4 = arm_group.go()
    print("x={},y={},z={},plan2={}".format(coke[0],coke[1],coke[2],plan4))
plan1=False
while(plan1==False):
    hand_group.set_named_target("close_coke")
    plan1 = hand_group.go()
plan4=False
while(plan4==False):
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = 0.0
    pose_target.orientation.x = 0.0
    pose_target.orientation.y = -0.972
    pose_target.orientation.z = 0.234
    #pose_target.position = translation 
    pose_target.position.x = coke[0] 
    pose_target.position.y = coke[1] -0.40
    pose_target.position.z = coke[2] +0.2
    arm_group.set_goal_tolerance(0.01)  
    arm_group.set_pose_target(pose_target)
    plan4 = arm_group.go()
    print("x={},y={},z={},plan3={}".format(coke[0],coke[1],coke[2],plan4))

plan1=False
while(plan1==False):
    arm_group.set_named_target("drop")
    plan1 = arm_group.go()

plan1=False
while(plan1==False):
    hand_group.set_named_target("open")
    plan1 = hand_group.go()

plan1=False
while(plan1==False):
    hand_group.set_named_target("close")
    plan1 = hand_group.go()

plan1=False
while(plan1==False):
    hand_group.set_named_target("open")
    plan1 = hand_group.go()    
#arm_group.set_named_target("photo")
#plan1 = arm_group.go()
plan4=False
while(plan4==False):
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = 0.0
    pose_target.orientation.x = 0.0
    pose_target.orientation.y = -0.972
    pose_target.orientation.z = 0.234
    #pose_target.position = translation 
    pose_target.position.x = battery[0] -0.005
    pose_target.position.y = battery[1] -0.40
    pose_target.position.z = battery[2] +0.09
    arm_group.set_pose_target(pose_target)
    arm_group.set_goal_tolerance(0.01)              
    plan4 = arm_group.go()
    print("x={},y={},z={},plan1={}".format(battery[0],battery[1],battery[2],plan4))
plan4=False
while(plan4==False):  
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = 0.0
    pose_target.orientation.x = 0.0
    pose_target.orientation.y = -0.972
    pose_target.orientation.z = 0.234
    #pose_target.position = translation 
    pose_target.position.x = battery[0] -0.005
    pose_target.position.y = battery[1] -0.184
    pose_target.position.z = battery[2] +0.09
    arm_group.set_goal_tolerance(0.01)  
    arm_group.set_pose_target(pose_target)              
    plan4 = arm_group.go()
    print("x={},y={},z={},plan2={}".format(battery[0],battery[1],battery[2],plan4))

plan1=False
while(plan1==False):
    hand_group.set_named_target("close_battery")
    plan1 = hand_group.go()
plan4=False
while(plan4==False):
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = 0.0
    pose_target.orientation.x = 0.0
    pose_target.orientation.y = -0.972
    pose_target.orientation.z = 0.234
    #pose_target.position = translation 
    pose_target.position.x = battery[0] -0.005
    pose_target.position.y = battery[1] -0.40
    pose_target.position.z = battery[2] + 0.2
    arm_group.set_goal_tolerance(0.01)  
    arm_group.set_pose_target(pose_target)
    plan4 = arm_group.go()
    print("x={},y={},z={},plan3={}".format(battery[0],battery[1],battery[2],plan4))

plan1=False
while(plan1==False):
    arm_group.set_named_target("drop")
    plan1 = arm_group.go()
plan1=False
while(plan1==False):
    hand_group.set_named_target("open")
    plan1 = hand_group.go()
plan1=False
while(plan1==False):
    hand_group.set_named_target("close")
    plan1 = hand_group.go()

plan1=False
while(plan1==False):
    hand_group.set_named_target("open")
    plan1 = hand_group.go()    
           
plan4=False

# #arm_group.set_named_target("photo")
# #plan1 = arm_group.go()

while(plan4==False):
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = 0.0
    pose_target.orientation.x = 0.0
    pose_target.orientation.y = -0.972
    pose_target.orientation.z = 0.234
    #pose_target.position = translation 
    pose_target.position.x = glue[0] -0.008
    pose_target.position.y = glue[1] -0.40
    pose_target.position.z = glue[2] +0.1
    arm_group.set_pose_target(pose_target)
    arm_group.set_goal_tolerance(0.01)              
    plan4 = arm_group.go()
    print("x={},y={},z={},plan1={}".format(glue[0],glue[1],glue[2],plan4))
plan4=False
while(plan4==False):  
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = 0.0
    pose_target.orientation.x = 0.0
    pose_target.orientation.y = -0.972
    pose_target.orientation.z = 0.234
    #pose_target.position = translation 
    pose_target.position.x = glue[0] -0.008
    pose_target.position.y = glue[1] -0.195
    pose_target.position.z = glue[2] +0.1
    arm_group.set_goal_tolerance(0.01)  
    arm_group.set_pose_target(pose_target)              
    plan4 = arm_group.go()
    print("x={},y={},z={},plan2={}".format(glue[0],glue[1],glue[2],plan4))

plan1=False
while(plan1==False):
    hand_group.set_named_target("close_glue")
    plan1 = hand_group.go()
plan4=False
while(plan4==False):
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = 0.0
    pose_target.orientation.x = 0.0
    pose_target.orientation.y = -0.972
    pose_target.orientation.z = 0.234
    #pose_target.position = translation 
    pose_target.position.x = glue[0] -0.005
    pose_target.position.y = glue[1] -0.40
    pose_target.position.z = glue[2] +0.2
    arm_group.set_goal_tolerance(0.01)  
    arm_group.set_pose_target(pose_target)
    plan4 = arm_group.go()
    print("x={},y={},z={},plan3={}".format(glue[0],glue[1],glue[2],plan4))

plan1=False
while(plan1==False):
    arm_group.set_named_target("drop")
    plan1 = arm_group.go()

plan1=False
while(plan1==False):  
    hand_group.set_named_target("open")
    plan1 = hand_group.go()

plan1=False
while(plan1==False):
    hand_group.set_named_target("close")
    plan1 = hand_group.go()

plan1=False
while(plan1==False):
    hand_group.set_named_target("open")
    plan1 = hand_group.go()        

rospy.sleep(5)
moveit_commander.roscpp_shutdown()
