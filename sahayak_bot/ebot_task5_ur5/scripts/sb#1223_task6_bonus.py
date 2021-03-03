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
from colorama import Fore, Back, Style

#declaring global variables
global coke_target,battery_target,fgpa_target,glue_target,arm_group,hand_group
coke_target=[0.0,0.0,0.0]
fgpa_target=[0.0,0.0,0.0]
glue_target=[0.0,0.0,0.0]
battery_target = [0.0,0.0,0.0]

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

def armPlanner2(object_position):
    baseArmPlanner(object_position, [-0.033, 0.206, 0.940, -0.269])

def armPlanner(object_position):
    baseArmPlanner(object_position, [0.0, 0.0, -0.972, 0.234])

def getObjCordinates(obj):
    t=TransformListener()
    # Detect object with target_frame obj
    t.waitForTransform("/ebot_base", obj, rospy.Time(), rospy.Duration(4.0))
    # Find Coordinate of the object
    (objCordinates,rotation1) = t.lookupTransform("/ebot_base", obj, rospy.Time())
    return objCordinates

def batteryArm(battery):
     # Move arm in front of battery
    armPlanner([battery[0] - 0.005, battery[1] - 0.40, battery[2] + 0.09])
    
    # Move arm to grab battery
    armPlanner([battery[0] - 0.005, battery[1] - 0.184, battery[2] + 0.09])
        
    # Move gripper to close_battery pose
    gripperPose("close_battery") 
    myPrint("Battery Picked")
    os.system('rosservice call /move_base/clear_costmaps "{}"')
    # Move arm back
    armPlanner([battery[0] - 0.005, battery[1] - 0.40, battery[2] + 0.2])
    armPose("travel2")

def adhesiveArm(adhesive):
     # Move arm in front of battery
    armPlanner([adhesive[0] - 0.005, adhesive[1] - 0.40, adhesive[2] + 0.09])
    
    # Move arm to grab battery
    armPlanner([adhesive[0] - 0.005, adhesive[1] - 0.184, adhesive[2] + 0.09])
        
    # Move gripper to close_battery pose
    gripperPose("close_adhesive") 
    myPrint("Adhesive Picked")
    os.system('rosservice call /move_base/clear_costmaps "{}"')
    # Move arm back
    armPlanner([adhesive[0] - 0.005, adhesive[1] - 0.40, adhesive[2] + 0.2])
    armPose("test")

def fpgaArm(fgpa):
    # Move arm in front of fgpa
    armPlanner2([fgpa[0] - 0.15, fgpa[1] - 0.3, fgpa[2] + 0.2])
    
    # Move arm to grab fgpa
    armPlanner2([fgpa[0] - 0.13, fgpa[1] - 0.18, fgpa[2] + 0.15])

    # Move gripper to close_fgpa pose
    gripperPose("close_fgpa")
    os.system('rosservice call /move_base/clear_costmaps "{}"')
    myPrint("FPGA board picked")

def glueArm(glue):
    # Move arm in front of glue
    armPlanner([glue[0] - 0.008, glue[1] - 0.4, glue[2] + 0.1])

    # Move arm to grab glue
    armPlanner([glue[0] - 0.008, glue[1] - 0.195, glue[2] + 0.1])

    # Move gripper to close_glue pose
    gripperPose("close_glue")
    myPrint("Glue Picked")

    os.system('rosservice call /move_base/clear_costmaps "{}"')
    # Move arm back
    armPlanner([glue[0] - 0.008, glue[1] - 0.195, glue[2] + 0.2])

def cokeArm(coke):
    # Move arm in front of coke
    armPlanner([coke[0], coke[1] - 0.4, coke[2] + 0.1])
    
    # Move arm to grab coke
    armPlanner([coke[0], coke[1] - 0.1836, coke[2] + 0.1])
        
    # Move gripper to close_coke pose
    gripperPose("close_coke") 
    myPrint("Coke Picked")
    os.system('rosservice call /move_base/clear_costmaps "{}"')
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

    # Define the goto function to move the bot
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
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.sleep(1)

# function to print with different colour
def myPrint(string):
    print(Fore.WHITE + Back.GREEN + string + Style.RESET_ALL)

def bot_driver():
    # Initializes the ROS node for the process.
    rospy.init_node('nav', anonymous=False)
    # Make an object of GoToPose
    navigator = GoToPose()
    # Make an object of TransformListener
    t = TransformListener() 
    myPrint("Started Run!")
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    hand_group = moveit_commander.MoveGroupCommander("grip_planning_group")
    arm_group = moveit_commander.MoveGroupCommander("arm_planning_group")
    
    # Move arm to travel2 pose
    armPose("test")
    # Cordinates of Waypoint 1
    position = {'x': 5.603923, 'y' : 5.035788}
    quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 0.9999619694705243, 'r4' : 0.008721216235724293}
    frequency = 200
    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)
    
    # Clear costmaps
    os.system('rosservice call /move_base/clear_costmaps "{}"')
    armPose("travel2")
    # Move arm to photo8 pose
    armPose("photo8")
    # open find_object_3d_session in new terminal tab
    os.system("gnome-terminal --tab -- roslaunch my_object_recognition_pkg start_find_object_3d_session.launch")   
    rospy.sleep(0.1)
    try:
        try:
            # Get battery cordinates
            adhesive = getObjCordinates("/object_159")
        except:
            # Get battery cordinates 
            adhesive = getObjCordinates("/object_163") 
    except:
        print("objectnotfound")  
        # Move arm to photo3 pose 
        armPose("photo3")
        try:
            try:
                # Get battery cordinates
                adhesive = getObjCordinates("/object_159")
            except:
                # Get battery cordinates
                adhesive = getObjCordinates("/object_163") 
        except:
            print("objectnotfound") 
            # Move arm to photo4 pose
            armPose("photo4") 
            try:
                try:
                    # Get battery cordinates
                    adhesive = getObjCordinates("/object_159")
                except:
                    # Get battery cordinates
                    adhesive = getObjCordinates("/object_163")       
            except:
                print("objectnotfound")
            else:
                adhesiveArm(adhesive)        
        else:
            # Move arm to photo4 pose
            armPose("photo4")
            adhesiveArm(adhesive) 
    else:
        # Move arm to photo3 pose to detect middle objects
        armPose("photo3")
        # Move arm to photo4 pose to detect all objects 
        armPose("photo4")
        adhesiveArm(adhesive) 
    # shutdown all terminal instances to close find_object_3d_session gui
    os.system("pkill gnome-terminal")
    # Move arm to travel2 pose     
    armPose("test")
    gripperPose("open")



# Python Main
if __name__ == '__main__':
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        hand_group = moveit_commander.MoveGroupCommander("grip_planning_group")
        arm_group = moveit_commander.MoveGroupCommander("arm_planning_group")
        bot_driver()
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C Quit")
        