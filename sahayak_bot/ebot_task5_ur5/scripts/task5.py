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
        pose_target.orientation.w = -0.033
        pose_target.orientation.x = 0.206
        pose_target.orientation.y = 0.940
        pose_target.orientation.z = -0.269
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

def handle_result(result,position):
    # If bot reached destination then print the cordinates of bot 
    if result:
        rospy.loginfo("I'm here now... (%s, %s) pose", position['x'], position['y'])
    # If bot did not reach destination then print failure message 
    else:
        rospy.loginfo("The base failed to reach the desired pose")
        

# Define class named GoToPose
class GoToPose():
    def __init__(self):

        self.goal_sent = False

        rospy.on_shutdown(self.shutdown)

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")

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
            print("GOAL SUCCEED")
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
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
    
    rospy.sleep(5)
    plan1 = False
    while(plan1==False):
        arm_group.set_named_target("travel2")
        plan1 = arm_group.go()
    # Cordinates of Waypoint 1
    position = {'x': 14.6058803, 'y' : -0.802476}
    quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : -0.707, 'r4' : 0.707}
    frequency = 200

    # Print Cordinates to Console
    rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)
    handle_result(result, position)

    os.system('rosservice call /move_base/clear_costmaps "{}"')
    plan1 = False
    while(plan1==False):
        arm_group.set_named_target("photo")
        plan1 = arm_group.go()     
    rospy.sleep(20)    
    rospy.loginfo("Pantry Reached")
    #t.waitForTransform("/ebot_base", "/object_139", rospy.Time(), rospy.Duration(4.0)) ###COKE
    if t.frameExists("object_143") :
        #Detect and assign coke cordinates using two possible object orientations
        # try:
        t.waitForTransform("/ebot_base", "/object_143", rospy.Time(), rospy.Duration(4.0))
        (coke,rotation1) = t.lookupTransform("/ebot_base", "/object_143", rospy.Time())
        # except:
        #     t.waitForTransform("/ebot_base", "/object_133", rospy.Time(), rospy.Duration(4.0))
        #     (coke,rotation1) = t.lookupTransform("/ebot_base", "/object_133", rospy.Time())    
        # Kill object detection nodes after successfull detection
        # os.system("rosnode kill "+ '/find_object_3d')
        # os.system("rosnode kill "+ '/tf_example')    
            # Tune cordinates
        coke_target[0] = coke[0]
        coke_target[1] = coke[1] - 0.4
        coke_target[2] = coke[2] + 0.1
        # Move arm in front of coke
        armPlanner(coke_target)
        
        # Tune cordinates
        coke_target[0] = coke[0]
        coke_target[1] = coke[1] - 0.1836
        coke_target[2] = coke[2] + 0.1
        # Move arm to grab coke
        armPlanner(coke_target)
        
        # Move gripper to close_coke pose
        gripperPose("close_coke")
        rospy.sleep(1) 
        rospy.loginfo("Coke Picked")
        # Tune cordinates
        coke_target[0] = coke[0]
        coke_target[1] = coke[1] - 0.40
        coke_target[2] = coke[2] + 0.2
        # Move arm back
        armPlanner(coke_target) 
        plan1 = False
        while(plan1==False):
            arm_group.set_named_target("travel2")
            plan1 = arm_group.go() 
    else:        
        plan1 = False
        while(plan1==False):
            arm_group.set_named_target("travel2")
            plan1 = arm_group.go()
        # Cordinates of Waypoint 1
        position = {'x': 11.21183 , 'y' : -1.307573}
        quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 0.739, 'r4' : 0.674}
        frequency = 60
        # Print Cordinates to Console
        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        # Bot reached destination or not
        result = navigator.goto(position, quaternion, frequency)
        handle_result(result, position)

        os.system('rosservice call /move_base/clear_costmaps "{}"')

        plan1 = False
        while(plan1==False):
            arm_group.set_named_target("photo")
            plan1 = arm_group.go()
        rospy.sleep(20) 
        #Detect and assign coke cordinates using two possible object orientations
        try:
            t.waitForTransform("/ebot_base", "/object_145", rospy.Time(), rospy.Duration(4.0))
            (coke,rotation1) = t.lookupTransform("/ebot_base", "/object_145", rospy.Time())
        except:
            t.waitForTransform("/ebot_base", "/object_133", rospy.Time(), rospy.Duration(4.0))
            (coke,rotation1) = t.lookupTransform("/ebot_base", "/object_133", rospy.Time())    
        # Kill object detection nodes after successfull detection
        # os.system("rosnode kill "+ '/find_object_3d')
        # os.system("rosnode kill "+ '/tf_example')    
            # Tune cordinates
        coke_target[0] = coke[0]
        coke_target[1] = coke[1] - 0.4
        coke_target[2] = coke[2] + 0.1
        # Move arm in front of coke
        armPlanner(coke_target)
        
        # Tune cordinates
        coke_target[0] = coke[0]
        coke_target[1] = coke[1] - 0.1836
        coke_target[2] = coke[2] + 0.1
        # Move arm to grab coke
        armPlanner(coke_target)
         
        # Move gripper to close_coke pose
        gripperPose("close_coke") 
        rospy.sleep(1)
        rospy.loginfo("Coke Picked")
        # Tune cordinates
        coke_target[0] = coke[0]
        coke_target[1] = coke[1] - 0.40
        coke_target[2] = coke[2] + 0.2
        # Move arm back
        armPlanner(coke_target)
        plan1 = False
        while(plan1==False):
            arm_group.set_named_target("travel2")
            plan1 = arm_group.go() 
    # Cordinates of Waypoint 1
    position = {'x': 7.00, 'y' : 2.6}
    quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 0.043, 'r4' :0.999 }############ TO DO
    frequency = 200

    # Print Cordinates to Console
    rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)
    handle_result(result, position)

    os.system('rosservice call /move_base/clear_costmaps "{}"')
    plan1 = False
    while(plan1==False):
        arm_group.set_named_target("drop_right")
        plan1 = arm_group.go()  
    gripperPose("open")
    rospy.sleep(1)
    rospy.loginfo("Meeting Room Reached")  
    
    rospy.loginfo("Coke Dropped in DropBox2")  
    plan1 = False
    while(plan1==False):
        arm_group.set_named_target("travel2")
        plan1 = arm_group.go()  
    # Cordinates of Waypoint 2
    position = {'x': 7.76, 'y' : 2.4}
    quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 0.043, 'r4' :0.999}  
    frequency = 200

    # Print Cordinates to Console
    rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)
    handle_result(result, position)

    os.system('rosservice call /move_base/clear_costmaps "{}"') 
    plan1 = False
    while(plan1==False):
        arm_group.set_named_target("photo")
        plan1 = arm_group.go()
    rospy.sleep(20)        
    t.waitForTransform("/ebot_base", "/object_149", rospy.Time(), rospy.Duration(4.0))
    (glue,rotation2) = t.lookupTransform("/ebot_base", "/object_149", rospy.Time())

    # Kill object detection nodes after successfull detection
    # os.system("rosnode kill "+ '/find_object_3d')
    # os.system("rosnode kill "+ '/tf_example') 

    # Tune cordinates
    glue_target[0] = glue[0] - 0.008
    glue_target[1] = glue[1] - 0.40
    glue_target[2] = glue[2] + 0.1
    # Move arm in front of glue
    armPlanner(glue_target)
    
    # Tune cordinates
    glue_target[0] = glue[0] - 0.008
    glue_target[1] = glue[1] - 0.195
    glue_target[2] = glue[2] + 0.1
    # Move arm to grab glue
    armPlanner(glue_target)

    # Move gripper to close_glue pose
    gripperPose("close_glue")
    rospy.sleep(1)
    rospy.loginfo("Glue Picked")

    # Tune cordinates
    glue_target[0] = glue[0] - 0.008
    glue_target[1] = glue[1] - 0.195
    glue_target[2] = glue[2] + 0.2
    os.system('rosservice call /move_base/clear_costmaps "{}"')
    # Move arm back
    armPlanner(glue_target)
    plan1 = False
    while(plan1==False):
        arm_group.set_named_target("travel2")
        plan1 = arm_group.go()
    # os.system('rosservice call /move_base/clear_costmaps "{}"')    
    # Cordinates of Waypoint 2
    position = {'x': 10.9, 'y' : 9.73}
    quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 0.707, 'r4' : 0.707}
    frequency = 200
    # Print Cordinates to Console
    rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)
    handle_result(result, position)

    os.system('rosservice call /move_base/clear_costmaps "{}"')
    plan1 = False
    while(plan1==False):
        arm_group.set_named_target("drop_left")
        plan1 = arm_group.go()  
    gripperPose("open")
    rospy.sleep(1)
    rospy.loginfo("Research Lab Reached")  
    rospy.loginfo("Glue Dropped in DropBox3")

    plan1 = False
    while(plan1==False):
        arm_group.set_named_target("travel2")
        plan1 = arm_group.go()    
    # Cordinates of Waypoint 2
    position = {'x': 25.948754, 'y' : -3.002912}
    quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : -0.894, 'r4' : 0.449}
    frequency = 200
    # Print Cordinates to Console
    rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)
    handle_result(result, position)

    os.system('rosservice call /move_base/clear_costmaps "{}"')
    rospy.loginfo("Store Room Reached") 
    plan1 = False
    while(plan1==False):
        arm_group.set_named_target("photo")
        plan1 = arm_group.go()
    rospy.sleep(20)
    t.waitForTransform("/ebot_base", "/object_155", rospy.Time(), rospy.Duration(4.0))
    (fgpa,rotation3) = t.lookupTransform("/ebot_base", "/object_155", rospy.Time())

    # Kill object detection nodes after successfull detection
    # os.system("rosnode kill "+ '/find_object_3d')
    # os.system("rosnode kill "+ '/tf_example') 

    # Tune cordinates
    fgpa_target[0] = fgpa[0] -0.13
    fgpa_target[1] = fgpa[1] -0.2
    fgpa_target[2] = fgpa[2] +0.2
    # Move arm in front of glue
    armPlanner2(fgpa_target)
    print("done")
    # Tune cordinates
    fgpa_target[0] = fgpa[0] - 0.13
    fgpa_target[1] = fgpa[1] - 0.18
    fgpa_target[2] = fgpa[2] + 0.1
    # Move arm to grab glue
    armPlanner2(fgpa_target)

    # Move gripper to close_glue pose
    gripperPose("close_coke")
    rospy.sleep(1)
    # # Tune cordinates
    # fgpa_target[0] = fgpa[0] - 0.008
    # fgpa_target[1] = fgpa[1] - 0.195
    # fgpa_target[2] = fgpa[2] + 0.3
    # # Move arm back
    # armPlanner2(fgpa_target)     
    rospy.loginfo("FPGA board picked")
    ######################
    #######################
    plan1 = False
    while(plan1==False):
        arm_group.set_named_target("travel2")
        plan1 = arm_group.go()
    # Cordinates of Waypoint 2
    position = {'x': 5.61, 'y' : -0.574539}
    quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 0.081, 'r4' : 0.997}
    frequency = 200
    # Print Cordinates to Console
    rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)
    handle_result(result, position)

    os.system('rosservice call /move_base/clear_costmaps "{}"')
    plan1 = False
    while(plan1==False):
        arm_group.set_named_target("drop_left")
        plan1 = arm_group.go()  
    gripperPose("open")
    rospy.loginfo("Conference Room Reached");
    rospy.loginfo("FPGA board dropped in Dropbox1")     
    
    plan1 = False
    while(plan1==False):
        arm_group.set_named_target("travel2")
        plan1 = arm_group.go()
    # Cordinates of Waypoint 2
    position = {'x': 0.01, 'y' : 0.01}
    quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 0.081, 'r4' : 0.997}
    frequency = 200
    # Print Cordinates to Console
    rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)
    handle_result(result, position)
    rospy.loginfo("Mission Accomplished!");


# Python Main
if __name__ == '__main__':
    rospy.sleep(10)
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        hand_group = moveit_commander.MoveGroupCommander("grip_planning_group")
        arm_group = moveit_commander.MoveGroupCommander("arm_planning_group")
        bot_driver()
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C Quit")