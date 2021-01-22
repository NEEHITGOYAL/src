#! /usr/bin/env python
import sys
import os
import rospy
import moveit_commander
import geometry_msgs.msg
from tf import TransformListener
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from trajectory_msgs.msg import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from object_msgs.msg import ObjectPose
global coke_target,battery_target,glue_target
coke_target=[0.0,0.0,0.0]
battery_target=[0.0,0.0,0.0]
glue_target=[0.0,0.0,0.0]
def bot_driver():
        # Initializes the ROS node for the process.
    rospy.init_node('task4_1223', anonymous=False)
    # Make an object of GoToPose
    navigator = GoToPose()
    
    rospy.sleep(1)
    # Cordinates of Waypoint 1
    position = {'x': 5, 'y' :-1.1}
    quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.336, 'r4' : 0.949}
    frequency = 60

    # Print Cordinates to Console
    rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)
    handle_result(result, position)

    position = {'x': 5.5, 'y' :-0.9}
    quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.336, 'r4' : 0.949}
    frequency = 60

    # Print Cordinates to Console
    rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)
    handle_result(result, position)
    rospy.sleep(1)
#######################################################################################
    global var_handle_pub
    var_handle_pub = rospy.Publisher('/detection_info', ObjectPose, queue_size=10)

    # Make an object of ObjectPose
    global msg 
    msg = ObjectPose()

    moveit_commander.roscpp_initialize(sys.argv)

    # Initializes the ROS node for the process.
    rospy.init_node('grasp_demo', anonymous=True)
    robot = moveit_commander.RobotCommander()

    # Put the arm in the start position
    global arm_group 
    arm_group = moveit_commander.MoveGroupCommander("arm_planning_group")
    
    # Put the grip in the start position
    global hand_group 
    hand_group = moveit_commander.MoveGroupCommander("grip_planning_group")

    # Move arm to photo pose.
    armPose("photo")

    # Move gripper to open pose
    gripperPose("open")

    t = TransformListener()

    # Wait for coke object to be detected 
    t.waitForTransform("/ebot_base", "/object_110", rospy.Time(), rospy.Duration(4.0))
    # Assign dected cordinates to coke list
    (coke,rotation1) = t.lookupTransform("/ebot_base", "/object_110", rospy.Time())
    
    #Detect and assign Battery cordinates using two possible object orientations
    try:
        t.waitForTransform("/ebot_base", "/object_109", rospy.Time(), rospy.Duration(4.0))
        (battery,rotation2) = t.lookupTransform("/ebot_base", "/object_109", rospy.Time())
    except:
        t.waitForTransform("/ebot_base", "/object_108", rospy.Time(), rospy.Duration(4.0))
        (battery,rotation2) = t.lookupTransform("/ebot_base", "/object_108", rospy.Time())   
    
    # Wait for glue object to be detected 
    t.waitForTransform("/ebot_base", "/object_106", rospy.Time(), rospy.Duration(4.0))
    # Assign dected cordinates to glue list
    (glue,rotation3) = t.lookupTransform("/ebot_base", "/object_106", rospy.Time())

    # Kill object detection nodes after successfull detection
    os.system("rosnode kill "+ '/find_object_3d')
    os.system("rosnode kill "+ '/tf_example')

    # Publish Cordinates of objects on detection_info node
    publishCordinates('Coke Can',coke)
    publishCordinates('Battery',battery)
    publishCordinates('Glue',glue)
    
    
    # Tune cordinates
    coke_target[0] = coke[0]
    coke_target[1] = coke[1] - 0.4
    coke_target[2] = coke[2] + 0.1
    # Move arm in front of coke
    armPlanner(coke_target)
    
    # Tune cordinates
    coke_target[0] = coke[0] 
    coke_target[1] = coke[1] - 0.183
    coke_target[2] = coke[2] + 0.1
    # Move arm to grab coke
    armPlanner(coke_target)
    
    # Move gripper to close_coke pose
    gripperPose("close_coke")
    rospy.sleep(1)
    # Tune cordinates
    coke_target[0] = coke[0]
    coke_target[1] = coke[1] - 0.40
    coke_target[2] = coke[2] + 0.2
    # Move arm back
    armPlanner(coke_target)

    # Move arm to drop pose
    armPose("drop")

    # Move gripper to open pose
    gripperPose("open")

    # Tune cordinates
    battery_target[0] = battery[0] 
    battery_target[1] = battery[1] - 0.40
    battery_target[2] = battery[2] + 0.09
    # Move arm in front of battery
    armPlanner(battery_target)

    # Tune cordinates
    battery_target[0] = battery[0] 
    battery_target[1] = battery[1] - 0.184
    battery_target[2] = battery[2] + 0.09
    # Move arm to grab battery
    armPlanner(battery_target)

    # Move gripper to close_battery pose
    gripperPose("close_battery")
    rospy.sleep(1)
    # Tune cordinates
    battery_target[0] = battery[0] 
    battery_target[1] = battery[1] - 0.40
    battery_target[2] = battery[2] + 0.2
    # Move arm back
    armPlanner(battery_target)

    # Move arm to drop pose
    armPose("drop")

    # Move gripper to open pose
    gripperPose("open")

    # Tune cordinates
    glue_target[0] = glue[0] 
    glue_target[1] = glue[1] - 0.40
    glue_target[2] = glue[2] + 0.1
    # Move arm in front of glue
    armPlanner(glue_target)
    
    # Tune cordinates
    glue_target[0] = glue[0] 
    glue_target[1] = glue[1] - 0.195
    glue_target[2] = glue[2] + 0.1
    # Move arm to grab glue
    armPlanner(glue_target)

    # Move gripper to close_glue pose
    gripperPose("close_glue")
    rospy.sleep(1)
    # Tune cordinates
    glue_target[0] = glue[0] 
    glue_target[1] = glue[1] - 0.195
    glue_target[2] = glue[2] + 0.1
    # Move arm back
    armPlanner(glue_target)

    # Move arm to drop pose
    armPose("drop")

    # Move gripper to open pose
    gripperPose("open")        

    rospy.sleep(5)
    moveit_commander.roscpp_shutdown()
##########################################################################################



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
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

def publishCordinates(name, object):
    # Name of the object
    msg.name = name
    # Position Cordinates of the object
    msg.pose.pose.position.x = object[0] 
    msg.pose.pose.position.y = object[1]
    msg.pose.pose.position.z = object[2]
    # Publish the object
    var_handle_pub.publish(msg)
    print(name, ' published')

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
    try:
        bot_driver()
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C Quit")
