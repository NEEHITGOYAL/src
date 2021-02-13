#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import sys
import os
import moveit_commander
import geometry_msgs.msg
from actionlib_msgs.msg import *
from trajectory_msgs.msg import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from tf import TransformListener

global coke_target,fgpa_target,glue_target,arm_group,hand_group
coke_target=[0.0,0.0,0.0]
fgpa_target=[0.0,0.0,0.0]
glue_target=[0.0,0.0,0.0]

def gripperPose(config):
    plan1 = False
    # Put the grip in the start position
    # hand_group = moveit_commander.MoveGroupCommander("grip_planning_group")
    # Try to planning until successfull
    while(plan1 == False):
        # Set Hand to given pose configuration
        hand_group.set_named_target(config)
        # Check if planning was successfull
        plan1 = hand_group.go()

def armPose(config):
    plan1 = False
    # Put the arm in the start position
    # arm_group = moveit_commander.MoveGroupCommander("arm_planning_group")
    # Try to planning until successfull
    while(plan1 == False):
        # Set Arm to given pose configuration
        arm_group.set_named_target(config)
        # Check if planning was successfull
        plan1 = arm_group.go()    


def armPlanner2(object_position):
    plan4 = False
    # Try to planning until successfull
    while(plan4 == False):
        pose_target = geometry_msgs.msg.Pose()
        # Fixed object orientation
        pose_target.orientation.w = 0.653
        pose_target.orientation.x = -0.653
        pose_target.orientation.y = 0.272
        pose_target.orientation.z = -0.272
        # Target object position
        pose_target.position.x = object_position[0]
        pose_target.position.y = object_position[1]
        pose_target.position.z = object_position[2]
        arm_group.set_pose_target(pose_target)
        arm_group.set_goal_tolerance(0.01)
        # Check if planning was successfull              
        plan4 = arm_group.go()
        # print the cordinates of object
        print("x = {},y = {},z = {},plan = {}".format(object_position[0], object_position[1], object_position[2], plan4))


def armPlanner(object_position):
    plan4 = False
    # Try to planning until successfull
    while(plan4 == False):
        pose_target = geometry_msgs.msg.Pose()
        # Fixed object orientation
        pose_target.orientation.w = 0.0
        pose_target.orientation.x = 0.0
        pose_target.orientation.y = -0.972
        pose_target.orientation.z = 0.234
        # Target object position
        pose_target.position.x = object_position[0]
        pose_target.position.y = object_position[1]
        pose_target.position.z = object_position[2]
        arm_group.set_pose_target(pose_target)
        arm_group.set_goal_tolerance(0.01)
        # Check if planning was successfull              
        plan4 = arm_group.go()
        # print the cordinates of object
        print("x = {},y = {},z = {},plan = {}".format(object_position[0], object_position[1], object_position[2], plan4))

if __name__ == '__main__':   
        rospy.sleep(6)   
        print("start") 
        rospy.init_node('task4_gazebo_sim')
        attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        attach_srv.wait_for_service()
        rospy.loginfo('Applying brakes to ebot')
        req = AttachRequest()
        req.model_name_1 = 'ebot'
        req.link_name_1 = 'ebot_base'
        req.model_name_2 = 'ground_plane'
        req.link_name_2 = 'link'
        attach_srv.call(req)
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        hand_group = moveit_commander.MoveGroupCommander("grip_planning_group")
        arm_group = moveit_commander.MoveGroupCommander("arm_planning_group")
        plan1 = False
        while(plan1==False):
            arm_group.set_named_target("photo")
            plan1 = arm_group.go()
#         t = TransformListener()    
# ##############################################################################################################################################################################        
#         #  #Detect and assign coke cordinates using two possible object orientations
#         # try:
#         #     t.waitForTransform("/ebot_base", "/object_139", rospy.Time(), rospy.Duration(4.0))
#         #     (coke,rotation1) = t.lookupTransform("/ebot_base", "/object_139", rospy.Time())
#         # except:
#         #     t.waitForTransform("/ebot_base", "/object_133", rospy.Time(), rospy.Duration(4.0))
#         #     (coke,rotation1) = t.lookupTransform("/ebot_base", "/object_133", rospy.Time())    
#         # # Kill object detection nodes after successfull detection
#         # os.system("rosnode kill "+ '/find_object_3d')
#         # os.system("rosnode kill "+ '/tf_example')    
#         #  # Tune cordinates
#         # coke_target[0] = coke[0]
#         # coke_target[1] = coke[1] - 0.4
#         # coke_target[2] = coke[2] + 0.1
#         # # Move arm in front of coke
#         # armPlanner(coke_target)
        
#         # # Tune cordinates
#         # coke_target[0] = coke[0]
#         # coke_target[1] = coke[1] - 0.1836
#         # coke_target[2] = coke[2] + 0.1
#         # # Move arm to grab coke
#         # armPlanner(coke_target)
        
#         # # Move gripper to close_coke pose
#         # gripperPose("close_coke")

#         # # Tune cordinates
#         # coke_target[0] = coke[0]
#         # coke_target[1] = coke[1] - 0.40
#         # coke_target[2] = coke[2] + 0.2
#         # # Move arm back
#         # armPlanner(coke_target)

#         # # Move arm to drop pose
#         # armPose("drop")

#         # # Move gripper to open pose
#         # gripperPose("open")    
#         # gripperPose("close")

# #########################################################################################################################################################3

#         # t.waitForTransform("/ebot_base", "/object_132", rospy.Time(), rospy.Duration(4.0))
#         # (glue,rotation2) = t.lookupTransform("/ebot_base", "/object_132", rospy.Time())

#         # # Kill object detection nodes after successfull detection
#         # os.system("rosnode kill "+ '/find_object_3d')
#         # os.system("rosnode kill "+ '/tf_example') 

#         # # Tune cordinates
#         # glue_target[0] = glue[0] - 0.008
#         # glue_target[1] = glue[1] - 0.40
#         # glue_target[2] = glue[2] + 0.1
#         # # Move arm in front of glue
#         # armPlanner(glue_target)
        
#         # # Tune cordinates
#         # glue_target[0] = glue[0] - 0.008
#         # glue_target[1] = glue[1] - 0.195
#         # glue_target[2] = glue[2] + 0.1
#         # # Move arm to grab glue
#         # armPlanner(glue_target)

#         # # Move gripper to close_glue pose
#         # gripperPose("close_glue")

#         # # Tune cordinates
#         # glue_target[0] = glue[0] - 0.008
#         # glue_target[1] = glue[1] - 0.195
#         # glue_target[2] = glue[2] + 0.2
#         # # Move arm back
#         # armPlanner(glue_target)

#         # # Move arm to drop pose
#         # armPose("drop")

#         # # Move gripper to open pose
#         # gripperPose("open")  
#         # gripperPose("close")

# ########################################################################################################################################################################
#         t.waitForTransform("/ebot_base", "/object_131", rospy.Time(), rospy.Duration(4.0))
#         (fgpa,rotation3) = t.lookupTransform("/ebot_base", "/object_131", rospy.Time())

#         # Kill object detection nodes after successfull detection
#         os.system("rosnode kill "+ '/find_object_3d')
#         os.system("rosnode kill "+ '/tf_example') 

#         # Tune cordinates
#         fgpa_target[0] = fgpa[0] - 0.008
#         fgpa_target[1] = fgpa[1] - 0.195
#         fgpa_target[2] = fgpa[2] + 0.3
#         # Move arm in front of glue
#         armPlanner2(fgpa_target)
        
#         # Tune cordinates
#         fgpa_target[0] = fgpa[0] - 0.008
#         fgpa_target[1] = fgpa[1] - 0.195
#         fgpa_target[2] = fgpa[2] + 0.1
#         # Move arm to grab glue
#         armPlanner2(fgpa_target)

#         # Move gripper to close_glue pose
#         gripperPose("close_fgpa")

#         # Tune cordinates
#         fgpa_target[0] = fgpa[0] - 0.008
#         fgpa_target[1] = fgpa[1] - 0.195
#         fgpa_target[2] = fgpa[2] + 0.3
#         # Move arm back
#         armPlanner2(fgpa_target)

#         # Move arm to drop pose
#         armPose("drop")

#         # Move gripper to open pose
#         gripperPose("open")  
#         gripperPose("close")

# ########################################################################################################################################################################

#         rospy.sleep(5)
#         moveit_commander.roscpp_shutdown()      
