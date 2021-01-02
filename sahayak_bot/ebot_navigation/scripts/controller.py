#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from trajectory_msgs.msg import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

def bot_driver():

    # Initializes the ROS node for the process.
    rospy.init_node('nav_test', anonymous=False)
    # Make an object of GoToPose
    navigator = GoToPose()
    
    # Cordinates of Waypoint 1 
    position = {'x': -9.1, 'y' :-1.2}
    quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.336, 'r4' : 0.949}
    frequency = 60

    # Print Cordinates to Console
    rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)
    handle_result(result, position)

    # Cordinates of Waypoint 2
    position = {'x':10.7, 'y' :10.5}
    quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : -0.7, 'r4' : 0.7}
    frequency = 120

    # Print Cordinates to Console
    rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)
    handle_result(result, position)

    # Cordinates of Waypoint 3
    position = {'x':12.6, 'y' :-1.9}
    quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : -0.7, 'r4' : 0.7} # To be changed
    frequency = 60

    # Print Cordinates to Console
    rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)
    handle_result(result, position)

    # Cordinates of Waypoint 4
    position = {'x': 18.2, 'y' :-1.4}
    quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : -0.357, 'r4' : 0.934}
    frequency = 60

    # Print Cordinates to Console
    rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)
    handle_result(result, position)

    # Cordinates of Waypoint 5
    position = {'x': -2, 'y' : 4}
    quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.924, 'r4' : 0.381}
    frequency = 120

    # Print Cordinates to Console
    rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
    # Bot reached destination or not
    result = navigator.goto(position, quaternion, frequency)
    handle_result(result, position)

    rospy.sleep(1)

def handle_result(result):
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

# Python Main
if __name__ == '__main__':
        rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C Quit")
