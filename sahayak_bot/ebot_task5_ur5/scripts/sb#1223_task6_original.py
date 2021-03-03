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
        # print the cordinates of object
        #print("x = {},y = {},z = {},plan = {}".format(object_position[0], object_position[1], object_position[2], plan4))

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
    t=TransformListener()
    t.waitForTransform("/ebot_base", obj, rospy.Time(), rospy.Duration(4.0))
    (objCordinates,rotation1) = t.lookupTransform("/ebot_base", obj, rospy.Time())
    # Kill object detection nodes after successfull detection
    # os.system("pkill gnome-terminal")
    return objCordinates

def batteryArm(battery):
     # Move arm in front of coke
    armPlanner([battery[0] - 0.005, battery[1] - 0.40, battery[2] + 0.09])
    
    # Move arm to grab coke
    armPlanner([battery[0] - 0.005, battery[1] - 0.184, battery[2] + 0.09])
        
    # Move gripper to close_coke pose
    gripperPose("close_battery") 
    print("Battery Picked")
    os.system('rosservice call /move_base/clear_costmaps "{}"')
    # Move arm back
    armPlanner([battery[0] - 0.005, battery[1] - 0.40, battery[2] + 0.2])
    armPose("travel2")

def fpgaArm(fgpa):
    # Move arm in front of glue
    armPlanner2([fgpa[0] - 0.15, fgpa[1] - 0.3, fgpa[2] + 0.2])
    
    # Move arm to grab glue
    armPlanner2([fgpa[0] - 0.13, fgpa[1] - 0.18, fgpa[2] + 0.15])

    # Move gripper to close_glue pose
    gripperPose("close_fgpa")
    os.system('rosservice call /move_base/clear_costmaps "{}"')
    print("FPGA board picked")

def glueArm(glue):
    # Move arm in front of glue
        armPlanner([glue[0] - 0.008, glue[1] - 0.4, glue[2] + 0.1])

        # Move arm to grab glue
        armPlanner([glue[0] - 0.008, glue[1] - 0.195, glue[2] + 0.1])

        # Move gripper to close_glue pose
        gripperPose("close_glue")
        print("Glue Picked")

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
    print("Coke Picked")
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
    print("Started Run!")
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    hand_group = moveit_commander.MoveGroupCommander("grip_planning_group")
    arm_group = moveit_commander.MoveGroupCommander("arm_planning_group")
    
    armPose("travel2")
    # Cordinates of Waypoint 1
    position = {'x': 25.9509754, 'y' : -3.202912}
    quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : -0.894, 'r4' : 0.449}
    frequency = 200
    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)


    os.system('rosservice call /move_base/clear_costmaps "{}"')

    armPose("photo8")
    os.system("gnome-terminal --tab -- roslaunch my_object_recognition_pkg start_find_object_3d_session.launch")   
    rospy.sleep(0.1)
    try:
        try:
            battery = getObjCordinates("/object_162")
        except:
            battery = getObjCordinates("/object_157") 
    except:
        print("objectnotfound")   
        armPose("photo3")
        try:
            try:
                battery = getObjCordinates("/object_162")
            except:
                battery = getObjCordinates("/object_157") 
        except:
            print("objectnotfound") 
            armPose("photo4") 
            try:
                try:
                    battery = getObjCordinates("/object_162")
                except:
                    battery = getObjCordinates("/object_157")       
            except:
                print("objectnotfound")
            else:
                batteryArm(battery)        
        else:
            armPose("photo4")
            batteryArm(battery) 
    else:
        armPose("photo3")   
        armPose("photo4")
        batteryArm(battery) 
    os.system("pkill gnome-terminal")     
    armPose("travel2")
    
    # Cordinates of Waypoint 2
    position = {'x': 10.9, 'y' : 9.73}
    quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 0.707, 'r4' : 0.707}
    frequency = 200
    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)

    os.system('rosservice call /move_base/clear_costmaps "{}"')

    armPose("drop_left")
    gripperPose("open")
    print("Battery Dropped in DropBox3")

    armPose("travel2")   
    
    # Cordinates of Waypoint 3
    position = {'x': 14.7058803, 'y' : -0.802476}
    quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : -0.707, 'r4' : 0.707}
    frequency = 200

    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)

    os.system('rosservice call /move_base/clear_costmaps "{}"')

    armPose("photo")

    os.system("gnome-terminal --tab -- roslaunch my_object_recognition_pkg start_find_object_3d_session.launch")       
    #Detect and assign coke cordinates using two possible object orientations
    try:
        coke = getObjCordinates("/object_143") 
    except:
        print("objectnotfound")
        try:
            coke = getObjCordinates("/object_145")
        except:
           print("objectnotfound")
        else:
            cokeArm(coke)        
    else:      
        cokeArm(coke)
    os.system('rosservice call /move_base/clear_costmaps "{}"')    
    os.system("pkill gnome-terminal")
    armPose("travel2")
    # Cordinates of Waypoint 4
    position = {'x': 11.21183 , 'y' : -1.307573}
    quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 0.739, 'r4' : 0.674}
    frequency = 60
    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)

    os.system('rosservice call /move_base/clear_costmaps "{}"')

    armPose("photo")
    os.system("gnome-terminal --tab -- roslaunch my_object_recognition_pkg start_find_object_3d_session.launch")   

    #Detect and assign coke cordinates using two possible object orientations
    try:
        coke = getObjCordinates("/object_143") 
    except:
        print("objectnotfound")
        try:
            coke = getObjCordinates("/object_145")
        except:
            print("objectnotfound")
        else:
            cokeArm(coke)        
    else:      
        cokeArm(coke)
    # os.system('rosservice call /move_base/clear_costmaps "{}"')    
    os.system("pkill gnome-terminal")
    armPose("travel2") 

    # Cordinates of Waypoint 5
    position = {'x': 8.75551, 'y' : 2.8}
    quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 0.723988125006, 'r4' : 0.689812434543}
    frequency = 200
    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)

    # Cordinates of Waypoint 6
    position = {'x': 8.75551, 'y' : 2.8}
    quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : -0.686252207602, 'r4' : -0.686252207602}
    frequency = 200
    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)
    
    # Cordinates of Waypoint 7
    position = {'x': 5.61, 'y' : -0.574539}
    quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 0.081, 'r4' : 0.997}
    frequency = 200
    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)

    # os.system('rosservice call /move_base/clear_costmaps "{}"')

    armPose("drop_left")
    gripperPose("open")
    print("Coke dropped in Dropbox1")     
    
    armPose("travel2")
    # Cordinates of Waypoint 8
    position = {'x': 0.00, 'y' : 0.00}
    quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 0.081, 'r4' : 0.997}
    frequency = 200
    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)

    print("Mission Accomplished!");


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
        