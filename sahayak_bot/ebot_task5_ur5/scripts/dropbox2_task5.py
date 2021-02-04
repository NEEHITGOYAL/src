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

moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
hand_group = moveit_commander.MoveGroupCommander("grip_planning_group")
arm_group = moveit_commander.MoveGroupCommander("arm_planning_group")
arm_group.set_named_target("test")
plan1 = arm_group.go()

def bot_driver():

    # Initializes the ROS node for the process.
    rospy.init_node('nav', anonymous=False)
    # Make an object of GoToPose
    navigator = GoToPose()
    
    rospy.sleep(1)
    # Cordinates of Waypoint 1 
    position = {'x': 13.01, 'y' :-0.870855}
    quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : -0.7071045443232221, 'r4' : 0.7071090180427968}
    frequency = 60

    # Print Cordinates to Console
    rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)
    handle_result(result, position)

    ###############################################################################

                                 # PICK OBJECT

    ###############################################################################

    position = {'x': 13.01, 'y' :-0.870855}
    quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 0.7023450278590759, 'r4' : 0.7118366820006073}
    frequency = 60

    # Print Cordinates to Console
    rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)
    handle_result(result, position)

    os.system('rosservice call /move_base/clear_costmaps "{}"')

    #######    Moving to drop BOX 
    position = {'x': 7.00, 'y' : 2.681765}
    quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 0.999988965231997, 'r4' : 0.004697809515074247}
    frequency = 60

    # Print Cordinates to Console
    rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)
    handle_result(result, position)

    ###############################################################################

                        #  DROP OBJECT

    ###############################################################################

    position = {'x': 7.00, 'y' : 2.681765}
    quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 0.0, 'r4' : 1.0}
    frequency = 60

    # Print Cordinates to Console
    rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)
    handle_result(result, position)
    os.system('rosservice call /move_base/clear_costmaps "{}"')


    ################################################################################

                    #  PICK OBJECT

    ################################################################################
    
    position = {'x': 10.9, 'y' : 9.43}
    quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 0.7071045443232221, 'r4' : 0.7071090180427968}
    frequency = 60

    # Print Cordinates to Console
    rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)
    handle_result(result, position)


    position = {'x': 10.9, 'y' : 9.43}
    quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : -0.703534947165817, 'r4' : 0.7106606631271996}
    frequency = 60

    # Print Cordinates to Console
    rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)
    handle_result(result, position)
    os.system('rosservice call /move_base/clear_costmaps "{}"')


    position = {'x': 25.921013, 'y' : -2.823172}
    quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : -0.2830493561676561, 'r4' : 0.9591053445649624}
    frequency = 60

    # Print Cordinates to Console
    rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)
    handle_result(result, position)

    #rospy.sleep(3)
    # position = {'x': 6.990000, 'y' :2.761418}
    # quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 0.999991164579, 'r4' : -0.00420366082469}
    # frequency = 120

    # # Print Cordinates to Console
    # rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
    # # Bot reached destination or not
    # result = navigator.goto(position, quaternion, frequency)
    # handle_result(result, position)

    #rospy.sleep(1)

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

# Python Main
if __name__ == '__main__':
    try:
        bot_driver()
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C Quit")
