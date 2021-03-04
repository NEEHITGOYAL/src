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
    armPose("travel2")
    # #Cordinates of Waypoint 1
    # position = {'x': 25.9509754, 'y' : -3.202912}
    # quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : -0.894, 'r4' : 0.449}
    # frequency = 200
    # # Bot reached destination or not
    # result = navigator.goto(position, quaternion, frequency)
    
    # # Clear costmaps
    # os.system('rosservice call /move_base/clear_costmaps "{}"')
    # # Move arm to photo8 pose
    # armPose("photo8")
    # # open find_object_3d_session in new terminal tab
    # os.system("gnome-terminal --tab -- roslaunch my_object_recognition_pkg start_find_object_3d_session.launch")   
    # rospy.sleep(0.1)
    # try:
    #     try:
    #         # Get battery cordinates
    #         battery = getObjCordinates("/object_162")
    #     except:
    #         # Get battery cordinates 
    #         try:
    #             battery = getObjCordinates("/object_170") 
    #         except:
    #             battery = getObjCordinates("/object_157") 
    # except:
    #     print("objectnotfound")  
    #     # Move arm to photo3 pose 
    #     armPose("photo3")
    #     rospy.sleep(0.1)
    #     try:
    #         try:
    #             # Get battery cordinates
    #             battery = getObjCordinates("/object_162")
    #         except:
    #             try:
    #                 # Get battery cordinates
    #                 battery = getObjCordinates("/object_170")
    #             except:
    #                 # Get battery cordinates
    #                 battery = getObjCordinates("/object_157") 
    #     except:
    #         print("objectnotfound") 
    #         # Move arm to photo4 pose
    #         armPose("photo4")
    #         rospy.sleep(0.1) 
    #         try:
    #             try:
    #                 # Get battery cordinates
    #                 battery = getObjCordinates("/object_162")
    #             except:
    #                 try:
    #                 # Get battery cordinates
    #                     battery = getObjCordinates("/object_170")
    #                 except:
    #                 # Get battery cordinates
    #                     battery = getObjCordinates("/object_157")        
    #         except:
    #             print("objectnotfound")
    #         else:
    #             batteryArm(battery)        
    #     else:
    #         # Move arm to photo4 pose
    #         armPose("photo4")
    #         rospy.sleep(0.1)
    #         batteryArm(battery) 
    # else:
    #     # Move arm to photo3 pose to detect middle objects
    #     armPose("photo3")
    #     rospy.sleep(0.1)
    #     # Move arm to photo4 pose to detect all objects 
    #     armPose("photo4")
    #     rospy.sleep(0.1)
    #     batteryArm(battery) 
    # # shutdown all terminal instances to close find_object_3d_session gui
    # os.system("pkill gnome-terminal")
    # # Move arm to travel2 pose     
    # armPose("travel2")
    
    # # Cordinates of Waypoint 2
    # position = {'x': 10.9, 'y' : 9.73}
    # quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 0.707, 'r4' : 0.707}
    # frequency = 200
    # # Bot reached destination or not
    # result = navigator.goto(position, quaternion, frequency)

    # # os.system('rosservice call /move_base/clear_costmaps "{}"')
    # # Move arm to drop_left pose
    # armPose("drop_left")
    # # Move gripper to open pose
    # gripperPose("open")
    # myPrint("Battery Dropped in DropBox3")
    
    # # Move arm to travel2 pose
    # armPose("travel2")   
    
    # # Cordinates of Waypoint 3
    # position = {'x': 14.7058803, 'y' : -0.802476}
    # quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : -0.707, 'r4' : 0.707}
    # frequency = 200

    # # Bot reached destination or not
    # result = navigator.goto(position, quaternion, frequency)
    # # Clear costmaps
    # # os.system('rosservice call /move_base/clear_costmaps "{}"')
    # # Move arm to photo pose
    # armPose("photo")
    # # open find_object_3d_session in new terminal tab
    # os.system("gnome-terminal --tab -- roslaunch my_object_recognition_pkg start_find_object_3d_session.launch")  
    
    # # os.system('rosservice call /move_base/clear_costmaps "{}"')    
    # try:
    #     coke = getObjCordinates("/object_143") 
    # except:
    #     print(" ")
    # # shutdown all terminal instances to close find_object_3d_session gui
    # armPose("photo3")
    # # Move arm to travel2 pose
    # armPose("travel2")
    # os.system("pkill gnome-terminal")
    # # Cordinates of Waypoint 4
    # position = {'x': 11.21183 , 'y' : -1.307573}
    # quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 0.739, 'r4' : 0.674}
    # frequency = 60
    # # Bot reached destination or not
    # result = navigator.goto(position, quaternion, frequency)

    # # Clear costmaps
    # # os.system('rosservice call /move_base/clear_costmaps "{}"')
    # # Move arm to photo pose
    # armPose("photo")
    # # open find_object_3d_session in new terminal tab
    # os.system("gnome-terminal --tab -- roslaunch my_object_recognition_pkg start_find_object_3d_session.launch")
    # # Clear costmaps
    # # os.system('rosservice call /move_base/clear_costmaps "{}"')
    # try:
    #     coke = getObjCordinates("/object_143") 
    # except:
    #     print(" ")
    # armPose("photo3")
    # # Move arm to travel2 pose
    # armPose("travel2") 
    # # shutdown all terminal instances to close find_object_3d_session gui
    # os.system("pkill gnome-terminal")

    # # Cordinates of Waypoint 4
    # position = {'x': 11.31183 , 'y' : -1.307573}
    # quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 0.043, 'r4' :0.999 }
    # frequency = 60
    # # Bot reached destination or not
    # result = navigator.goto(position, quaternion, frequency)

    # Cordinates of Waypoint 5
    position = {'x': 8.75551, 'y' : 2.6}
    quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 0.723988125006, 'r4' : 0.689812434543}
    frequency = 200
    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)
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
    # Move arm to photo8 pose
    armPose("photo")
    # open find_object_3d_session in new terminal tab
    os.system("gnome-terminal --tab -- roslaunch my_object_recognition_pkg start_find_object_3d_session.launch")   
    rospy.sleep(0.1)
    glue_pick = True
    try:
        glue = getObjCordinates("/object_179")
    except:
        print("objectnotfound")
        try:
            glue = getObjCordinates("/object_171")
        except:
            print("objectnotfound")
            try:
                glue = getObjCordinates("/object_169")
            except:
                print("objectnotfound")
                try:
                    glue = getObjCordinates("/object_160")
                except:
                    print("objectnotfound")
                    try:
                        glue = getObjCordinates("/object_149")
                    except:
                        print("objectnotfound")
                        glue_pick = False
                    else:        
                        glueArm(glue)
                else:        
                    glueArm(glue)
            else:        
                glueArm(glue)
        else:        
            glueArm(glue)            
    else:        
        glueArm(glue)
        
    if (glue_pick == False):
        armPose("photo3")
        rospy.sleep(0.1)
        try:
            glue = getObjCordinates("/object_179")
        except:
            print("objectnotfound")
            try:
                glue = getObjCordinates("/object_171")
            except:
                print("objectnotfound")
                try:
                    glue = getObjCordinates("/object_169")
                except:
                    print("objectnotfound")
                    try:
                        glue = getObjCordinates("/object_160")
                    except:
                        print("objectnotfound")
                        try:
                            glue = getObjCordinates("/object_149")
                        except:
                            print("objectnotfound")
                            glue_pick = False
                        else:        
                            glueArm(glue)
                    else:        
                        glueArm(glue)
                else:        
                    glueArm(glue)
            else:        
                glueArm(glue)            
        else:        
            glueArm(glue)           

    os.system("pkill gnome-terminal")    
    armPose("test")

    # Cordinates of Waypoint 1
    position = {'x': 7.10, 'y' : 2.35}
    quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 0.043, 'r4' :0.999 }
    frequency = 200

    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)

    os.system('rosservice call /move_base/clear_costmaps "{}"')
    
    armPose("drop_right") 
    gripperPose("open")
    myPrint("Glue dropped in Dropbox2") 

    # Move arm to travel2 pose
    armPose("test") 
    
    # Cordinates of Waypoint 1
    position = {'x': 7.10, 'y' : 2.35}
    quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 0.739, 'r4' : 0.674}
    frequency = 200

    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)

    # Cordinates of Waypoint 1
    position = {'x': 5.603923, 'y' : 5.035788}
    quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 0.9999619694705243, 'r4' : 0.008721216235724293}
    frequency = 200
    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)
    
    # Clear costmaps
    os.system('rosservice call /move_base/clear_costmaps "{}"')
    # Move arm to photo8 pose
    armPose("photo")
    # open find_object_3d_session in new terminal tab
    os.system("gnome-terminal --tab -- roslaunch my_object_recognition_pkg start_find_object_3d_session.launch")   
    rospy.sleep(0.1)
    #Detect and assign coke cordinates using two possible object orientations
    adhesivePick = True
    try:
        adhesive = getObjCordinates("/object_180") 
    except:
        print("objectnotfound")
        try:
            adhesive = getObjCordinates("/object_159")
        except:
            print("objectnotfound")
            try:
                adhesive = getObjCordinates("/object_163")
            except:
                print("objectnotfound")
                adhesivePick = False
            else:
                adhesiveArm(adhesive) 
        else:
            adhesiveArm(adhesive)              
    else:      
        adhesiveArm(adhesive)
    if (adhesivePick == False): 
        try:
            adhesive = getObjCordinates("/object_180") 
        except:
            print("objectnotfound")
            try:
                adhesive = getObjCordinates("/object_159")
            except:
                print("objectnotfound")
                try:
                    adhesive = getObjCordinates("/object_163")
                except:
                    print("objectnotfound")
                    adhesivePick = False
                else:
                    adhesiveArm(adhesive) 
            else:
                adhesiveArm(adhesive)              
        else:      
            adhesiveArm(adhesive)        
    os.system("pkill gnome-terminal")
    # Move arm to travel2 pose     
    armPose("test")
    # Cordinates of Waypoint 6
    position = {'x': 8.75551, 'y' : 2.8}
    quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : -0.894, 'r4' : 0.449}
    frequency = 200
    result = navigator.goto(position, quaternion, frequency)
    armPose("travel2")
    # Cordinates of Waypoint 7
    position = {'x': 5.61, 'y' : -0.574539}
    quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 0.081, 'r4' : 0.997}
    frequency = 200
    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)
    os.system('rosservice call /move_base/clear_costmaps "{}"')
    # Move arm to drop_left pose
    armPose("drop_left")
    # Move gripper to open pose
    gripperPose("open")
    myPrint("Adhesive dropped in Dropbox1")  

    # Move arm to travel2 pose
    armPose("travel2")

    # Cordinates of Waypoint 8
    position = {'x': 5.61, 'y' : -0.574539}
    quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 0.723988125006, 'r4' : 0.689812434543}
    frequency = 20

    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)
    # Cordinates of Waypoint 9
    position = {'x': 0.00, 'y' : 0.00}
    quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 1.00, 'r4' : 0.0}
    frequency = 200
    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)
    os.system('rosservice call /move_base/clear_costmaps "{}"')

    myPrint("Mission Accomplished!")




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
        