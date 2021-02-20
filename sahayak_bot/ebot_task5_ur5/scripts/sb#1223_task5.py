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
from tf import TransformListener

#declaring global variables
global coke_target,fgpa_target,glue_target,arm_group,hand_group
coke_target=[0.0,0.0,0.0]
fgpa_target=[0.0,0.0,0.0]
glue_target=[0.0,0.0,0.0]

def gripperPose(config):
    plan1 = False
    # Put the grip in the start position
    hand_group = moveit_commander.MoveGroupCommander("grip_planning_group")
    # Try to planning until successfull
    while(plan1 == False):
        # Set Hand to given pose configuration
        hand_group.set_named_target(config)
        # Check if planning was successfull
        plan1 = hand_group.go()

def armPose(config):
    plan1 = False
    # Put the arm in the start position
    arm_group = moveit_commander.MoveGroupCommander("arm_planning_group")
    # Try to planning until successfull
    while(plan1 == False):
        # Set Arm to given pose configuration
        arm_group.set_named_target(config)
        # Check if planning was successfull
        plan1 = arm_group.go()    

def baseArmPlanner(object_position,object_orientation):
    plan4 = False
    arm_group = moveit_commander.MoveGroupCommander("arm_planning_group")
    # Try to planning until successfull
    while(plan4 == False):
        pose_target = geometry_msgs.msg.Pose()
        # Fixed object orientation
        pose_target.orientation.w = object_orientation[0]
        pose_target.orientation.x = object_orientation[1]
        pose_target.orientation.y = object_orientation[2]
        pose_target.orientation.z = object_orientation[3]
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

def armPlanner2(object_position):
    baseArmPlanner(object_position, [-0.033, 0.206, 0.940, -0.269])

def armPlanner(object_position):
    baseArmPlanner(object_position, [0.0, 0.0, -0.972, 0.234])

def handle_result(result,position):
    # If bot reached destination then print the cordinates of bot 
    if result:
        rospy.loginfo("I'm here now... (%s, %s) pose", position['x'], position['y'])
    # If bot did not reach destination then print failure message 
    else:
        rospy.loginfo("The base failed to reach the desired pose")

def getObjCordinates(obj):
    t.waitForTransform("/ebot_base", obj, rospy.Time(), rospy.Duration(4.0))
    (objCordinates,rotation1) = t.lookupTransform("/ebot_base", obj, rospy.Time())
    # Kill object detection nodes after successfull detection
    os.system("rosnode kill "+ '/find_object_3d')
    os.system("rosnode kill "+ '/tf_example')
    return objCordinates

# def detection(obj):
#     try:
#         t.waitForTransform("/ebot_base", obj, rospy.Time(), rospy.Duration(2.0))
#         (objCordinates,rotation1) = t.lookupTransform("/ebot_base", obj, rospy.Time())

#         if(obj=="/object_151" or obj=="/object_144"):
#             rospy.loginfo("Battery Detected")
#         elif(obj=="/object_145"):
#             rospy.loginfo("Adhesive Detected")
#         elif(obj=="/object_146"):
#             rospy.loginfo("Glass Detected")
#         elif(obj=="/object_149"):
#             rospy.loginfo("eYFI Board Detected")
#         elif(obj=="/object_150" or obj=="/object_152"):
#             rospy.loginfo("Pair of Wheels Package Detected")
#     except:
#         pass

def cokeArm(coke):
    # Move arm in front of coke
    armPlanner([coke[0], coke[1] - 0.4, coke[2] + 0.1])
    
    # Move arm to grab coke
    armPlanner([coke[0], coke[1] - 0.1836, coke[2] + 0.1])
        
    # Move gripper to close_coke pose
    gripperPose("close_coke") 
    rospy.sleep(0.1)
    rospy.loginfo("Coke Picked")

    # Move arm back
    armPlanner([coke[0], coke[1] - 0.4, coke[2] + 0.2])
    armPose("travel2")

# Define class named GoToPose
class GoToPose():
    def __init__(self):

        self.goal_sent = False

        rospy.on_shutdown(self.shutdown)

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        self.move_base.wait_for_server(rospy.Duration(5))

    def goto(self, pos, quat, t):

        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

        self.move_base.send_goal(goal)
        success = self.move_base.wait_for_result(rospy.Duration(t)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            result = True
            # print("GOAL SUCCEED")
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.sleep(1)

def bot_driver():
    # Initializes the ROS node for the process.
    rospy.init_node('nav', anonymous=False)
    # Make an object of GoToPose
    navigator = GoToPose()
    t = TransformListener() 
    rospy.loginfo("Started Run!")
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    hand_group = moveit_commander.MoveGroupCommander("grip_planning_group")
    arm_group = moveit_commander.MoveGroupCommander("arm_planning_group")
    
    armPose("travel2")

    # Cordinates of Waypoint 1
    position = {'x': 14.6058803, 'y' : -0.802476}
    quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : -0.707, 'r4' : 0.707}
    frequency = 200

    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)

    os.system('rosservice call /move_base/clear_costmaps "{}"')

    armPose("photo")

    os.system("gnome-terminal -- roslaunch my_object_recognition_pkg start_find_object_3d_session.launch")      
    rospy.sleep(3)    
    rospy.loginfo("Pantry Reached")
    # detection("/object_146")
    try: 
        cokeArm(getObjCordinates("/object_139"))
        rospy.loginfo("Coke Detected")
    except:
        try:     
            cokeArm(getObjCordinates("/object_133"))
            rospy.loginfo("Coke Detected")
        except:   
            os.system("rosnode kill "+ '/find_object_3d')
            os.system("rosnode kill "+ '/tf_example')     
   
            armPose("travel2")
            # Cordinates of Waypoint 1
            position = {'x': 11.21183 , 'y' : -1.307573}
            quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 0.739, 'r4' : 0.674}
            frequency = 60
            # Bot reached destination or not
            result = navigator.goto(position, quaternion, frequency)
        
            os.system('rosservice call /move_base/clear_costmaps "{}"')

            armPose("photo")
            os.system("gnome-terminal -- roslaunch my_object_recognition_pkg start_find_object_3d_session.launch")    
            rospy.sleep(3)

            #Detect and assign coke cordinates using two possible object orientations
            #detection("/object_146")
            try:
                coke = getObjCordinates("/object_139") 
                rospy.loginfo("Coke Detected")
            except:
                coke = getObjCordinates("/object_133")  
                rospy.loginfo("Coke Detected")
            cokeArm(coke)   

    # Cordinates of Waypoint 1
    position = {'x': 7.00, 'y' : 2.55}
    quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 0.043, 'r4' :0.999 }
    frequency = 200

    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)

    os.system('rosservice call /move_base/clear_costmaps "{}"')
    
    armPose("drop_right") 
    gripperPose("open")
    rospy.sleep(0.1)
    rospy.loginfo("Meeting Room Reached")  
    
    rospy.loginfo("Coke Dropped in DropBox2")  
  
    armPose("travel2")  
    # Cordinates of Waypoint 2
    position = {'x': 7.76, 'y' : 2.4}
    quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 0.043, 'r4' :0.999}  
    frequency = 200

    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)

    os.system('rosservice call /move_base/clear_costmaps "{}"') 

    armPose("photo")
    os.system("gnome-terminal -- roslaunch my_object_recognition_pkg start_find_object_3d_session.launch")    
    rospy.sleep(3)        
    # detection("/object_151")
    # detection("/object_144")
    # detection("/object_145")
    try:
        glue = getObjCordinates("/object_132")
        rospy.loginfo("Glue Detected")
    except:
        pass
    # Move arm in front of glue
    armPlanner([glue[0] - 0.008, glue[1] - 0.4, glue[2] + 0.1])

    # Move arm to grab glue
    armPlanner([glue[0] - 0.008, glue[1] - 0.195, glue[2] + 0.1])

    # Move gripper to close_glue pose
    gripperPose("close_glue")
    rospy.sleep(0.1)
    rospy.loginfo("Glue Picked")

    os.system('rosservice call /move_base/clear_costmaps "{}"')
    # Move arm back
    armPlanner([glue[0] - 0.008, glue[1] - 0.195, glue[2] + 0.2])

    armPose("travel2")
    
    position = {'x': 10.9, 'y' : 9.73}
    quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 0.707, 'r4' : 0.707}
    frequency = 200
    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)

    os.system('rosservice call /move_base/clear_costmaps "{}"')

    armPose("drop_left")
    gripperPose("open")
    rospy.sleep(0.1)
    rospy.loginfo("Research Lab Reached")  
    rospy.loginfo("Glue Dropped in DropBox3")

    armPose("travel2")   
    # Cordinates of Waypoint 2
    position = {'x': 25.948754, 'y' : -3.002912}
    quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : -0.894, 'r4' : 0.449}
    frequency = 200
    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)


    os.system('rosservice call /move_base/clear_costmaps "{}"')
    rospy.loginfo("Store Room Reached") 

    armPose("photo6")
    os.system("gnome-terminal -- roslaunch my_object_recognition_pkg start_find_object_3d_session.launch")    
    rospy.sleep(3)
    # detection("/object_150")
    # detection("/object_152")
    # detection("/object_149")
    # detection("/object_151")
    # detection("/object_144")
    # detection("/object_145")
    try:
        fgpa = getObjCordinates("/object_131")
        rospy.loginfo("FPGA Detected")
    except:
        pass

    # Move arm in front of glue
    armPlanner2([fgpa[0] - 0.15, fgpa[1] - 0.3, fgpa[2] + 0.2])
    
    # Move arm to grab glue
    armPlanner2([fgpa[0] - 0.13, fgpa[1] - 0.18, fgpa[2] + 0.15])

    # Move gripper to close_glue pose
    gripperPose("close_fgpa")
    rospy.sleep(0.1) 
    rospy.loginfo("FPGA board picked")
    
    armPose("travel2")
    # Cordinates of Waypoint 2
    position = {'x': 5.61, 'y' : -0.574539}
    quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 0.081, 'r4' : 0.997}
    frequency = 200
    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)

    os.system('rosservice call /move_base/clear_costmaps "{}"')

    armPose("drop_left")
    gripperPose("open")
    rospy.loginfo("Conference Room Reached");
    rospy.loginfo("FPGA board dropped in Dropbox1")     
    
    armPose("travel2")
    # Cordinates of Waypoint 2
    position = {'x': 0.01, 'y' : 0.01}
    quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 0.081, 'r4' : 0.997}
    frequency = 200
    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)

    rospy.loginfo("Mission Accomplished!");


# Python Main
if __name__ == '__main__':
    rospy.sleep(1)
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        hand_group = moveit_commander.MoveGroupCommander("grip_planning_group")
        arm_group = moveit_commander.MoveGroupCommander("arm_planning_group")
        bot_driver()
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C Quit")