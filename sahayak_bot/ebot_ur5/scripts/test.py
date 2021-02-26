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


#hand_group.set_named_target("Open")
#plan2 = hand_group.go()
plan4=False
while(plan4==False):
    print(" -180")
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = 0.707
    pose_target.orientation.x = -0.707
    pose_target.orientation.y = 0.0#-0.972
    pose_target.orientation.z = 0.0#0.234
    pose_target.position.x = 0.34
    pose_target.position.y = 0.61
    pose_target.position.z = 0.82
    arm_group.set_pose_target(pose_target)
    plan4 = arm_group.go()
rospy.sleep(5)    
plan4=False
while(plan4==False):
    print("-136")
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = 0.658
    pose_target.orientation.x = -0.658
    pose_target.orientation.y = -0.260#-0.972
    pose_target.orientation.z = 0.260#0.234
    pose_target.position.x = 0.34
    pose_target.position.y = 0.61
    pose_target.position.z = 0.82
    arm_group.set_pose_target(pose_target)
    plan4 = arm_group.go()   
rospy.sleep(5)     
plan4=False
while(plan4==False):
    print("-90")
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = 0.5
    pose_target.orientation.x = -0.5
    pose_target.orientation.y = -0.5#-0.972
    pose_target.orientation.z = 0.5#0.234
    pose_target.position.x = 0.34
    pose_target.position.y = 0.61
    pose_target.position.z = 0.82
    arm_group.set_pose_target(pose_target)
    plan4 = arm_group.go()  
rospy.sleep(5)
plan4=False
while(plan4==False):
    print("-46")
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = 0.277
    pose_target.orientation.x = -0.277
    pose_target.orientation.y = -0.651#-0.972
    pose_target.orientation.z = 0.651#0.234
    pose_target.position.x = 0.34
    pose_target.position.y = 0.61
    pose_target.position.z = 0.82
    arm_group.set_pose_target(pose_target)
    plan4 = arm_group.go()       
rospy.sleep(5)
plan4=False
while(plan4==False):
    print("0")
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = 0
    pose_target.orientation.x = 0
    pose_target.orientation.y = -0.707#-0.972
    pose_target.orientation.z = 0.707#0.234
    pose_target.position.x = 0.34
    pose_target.position.y = 0.61
    pose_target.position.z = 0.82
    arm_group.set_pose_target(pose_target)
    plan4 = arm_group.go()
rospy.sleep(5)
plan4=False
while(plan4==False):
    print("45")
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = -0.250
    pose_target.orientation.x = 0.250
    pose_target.orientation.y = -0.661#-0.972
    pose_target.orientation.z = 0.661#0.234
    pose_target.position.x = 0.34
    pose_target.position.y = 0.61
    pose_target.position.z = 0.82
    arm_group.set_pose_target(pose_target)
    plan4 = arm_group.go()    
rospy.sleep(5)    
plan4=False
while(plan4==False):
    print("90")
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = -0.5
    pose_target.orientation.x = 0.5
    pose_target.orientation.y = -0.5#-0.972
    pose_target.orientation.z = 0.5#0.234
    pose_target.position.x = 0.34
    pose_target.position.y = 0.61
    pose_target.position.z = 0.82
    arm_group.set_pose_target(pose_target)
    plan4 = arm_group.go()    
rospy.sleep(5)
plan4=False
while(plan4==False):
    print("135")
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = -0.655
    pose_target.orientation.x = 0.655
    pose_target.orientation.y = -0.267#-0.972
    pose_target.orientation.z = 0.267#0.234
    pose_target.position.x = 0.34
    pose_target.position.y = 0.61
    pose_target.position.z = 0.82
    arm_group.set_pose_target(pose_target)
    plan4 = arm_group.go()     
rospy.sleep(5)
plan4=False
while(plan4==False):
    print("180")
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = -0.707
    pose_target.orientation.x = 0.707
    pose_target.orientation.y = 0#-0.972
    pose_target.orientation.z = 0#0.234
    pose_target.position.x = 0.34
    pose_target.position.y = 0.61
    pose_target.position.z = 0.82
    arm_group.set_pose_target(pose_target)
    plan4 = arm_group.go()    
# plan4=False
# while(plan4==False):
#     pose_target = geometry_msgs.msg.Pose()
#     pose_target.orientation.w = 0
#     pose_target.orientation.x = 0
#     pose_target.orientation.y = 1#-0.972
#     pose_target.orientation.z = 0
#     pose_target.position.x = 0.34
#     pose_target.position.y = 0.61
#     pose_target.position.z = 0.82
#     arm_group.set_pose_target(pose_target)
#     plan4 = arm_group.go()
# plan4=False
# while(plan4==False):
#     pose_target = geometry_msgs.msg.Pose()
#     pose_target.orientation.w = 0
#     pose_target.orientation.x = 0
#     pose_target.orientation.y = 0.707#-0.972
#     pose_target.orientation.z = 0.707#0.234
#     pose_target.position.x = 0.34
#     pose_target.position.y = 0.61
#     pose_target.position.z = 0.82
#     arm_group.set_pose_target(pose_target)
#     plan4 = arm_group.go()
# plan4=False
# print("hhh")
# while(plan4==False):
#     print("hhh")
#     pose_target = geometry_msgs.msg.Pose()
#     pose_target.orientation.w = 0
#     pose_target.orientation.x = 0
#     pose_target.orientation.y = 0#-0.972
#     pose_target.orientation.z = 1#0.234
#     pose_target.position.x = 0.34
#     pose_target.position.y = 0.61
#     pose_target.position.z = 0.82
#     arm_group.set_pose_target(pose_target)
#     plan4 = arm_group.go()
# plan4=False
# while(plan4==False):
#     print("hhh")
#     pose_target = geometry_msgs.msg.Pose()
#     pose_target.orientation.w = 0
#     pose_target.orientation.x = 0
#     pose_target.orientation.y = -0.707#-0.972
#     pose_target.orientation.z = 0.707#0.234
#     pose_target.position.x = 0.34
#     pose_target.position.y = 0.61
#     pose_target.position.z = 0.82
#     arm_group.set_pose_target(pose_target)
#     plan4 = arm_group.go()    
# plan4=False
# while(plan4==False):
#     pose_target = geometry_msgs.msg.Pose()
#     pose_target.orientation.w = 0
#     pose_target.orientation.x = 0
#     pose_target.orientation.y = -0.972
#     pose_target.orientation.z = 0.234
#     pose_target.position.x = 0.34
#     pose_target.position.y = 0.61
#     pose_target.position.z = 0.82
#     arm_group.set_pose_target(pose_target)
#     plan4 = arm_group.go()
rospy.sleep(2)
moveit_commander.roscpp_shutdown()
