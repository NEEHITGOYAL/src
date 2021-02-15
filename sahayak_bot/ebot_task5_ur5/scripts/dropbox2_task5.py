#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from tf import TransformListener
import sys
import os
import moveit_commander
import geometry_msgs.msg
from actionlib_msgs.msg import *
from trajectory_msgs.msg import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
hand_group = moveit_commander.MoveGroupCommander("grip_planning_group")
arm_group = moveit_commander.MoveGroupCommander("arm_planning_group")
# plan1 = False
# while(plan1 == False):
#     arm_group.set_named_target("test")
#     plan1 = arm_group.go()

def bot_driver():

    # Initializes the ROS node for the process.
    rospy.init_node('nav', anonymous=False)
    # Make an object of GoToPose
    navigator = GoToPose()
    
    rospy.sleep(5)
    # Cordinates of Waypoint 1 
#     position = {'x': 13.01, 'y' :-0.870855}
#     quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : -0.7071045443232221, 'r4' : 0.7071090180427968}
#     frequency = 60

#     # Print Cordinates to Console
#     rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
#     # Bot reached destination or not
#     result = navigator.goto(position, quaternion, frequency)
#     handle_result(result, position)

#     os.system('rosservice call /move_base/clear_costmaps "{}"')

#     position = {'x': 14.534821, 'y' : -0.737697}
#     quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : -0.7199102499944642, 'r4' : 0.6940671667446228}
#     frequency = 60

#     # Print Cordinates to Console
#     rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
#     # Bot reached destination or not
#     result = navigator.goto(position, quaternion, frequency)
#     handle_result(result, position)

#     os.system('rosservice call /move_base/clear_costmaps "{}"')
# ##################################################################################

#                     ###  COKE GRIP

#     plan1 = False
#     while(plan1 == False):
#         arm_group.set_named_target("photo")
#         plan1 = arm_group.go()
#     t = TransformListener()
#     rospy.sleep(2)
#     #t.waitForTransform("/ebot_base", "/object_139", rospy.Time(), rospy.Duration(4.0)) ###COKE
#     if t.frameExists("object_143"):
#         t.waitForTransform("/ebot_base", "/object_143", rospy.Time(), rospy.Duration(4.0))
#         (coke,rotation) = t.lookupTransform("/ebot_base", "/object_143", rospy.Time())
#         plan4 = False
#         # Try to planning until successfull
#         while(plan4 == False):
#             pose_target = geometry_msgs.msg.Pose()
#             # Fixed object orientation
#             pose_target.orientation.w = 0.0
#             pose_target.orientation.x = 0.0
#             pose_target.orientation.y = -0.972
#             pose_target.orientation.z = 0.234
#             # Target object position
#             pose_target.position.x = coke[0]
#             pose_target.position.y = coke[1] - 0.25
#             pose_target.position.z = coke[2] + 0.2
#             arm_group.set_pose_target(pose_target)
#             arm_group.set_goal_tolerance(0.01)
#             # Check if planning was successfull              
#             plan4 = arm_group.go()
#             # print the cordinates of object
#             print("x = {},y = {},z = {},plan = {}".format(coke[0], coke[1], coke[2], plan4))
        
#         plan4 = False
#         while(plan4 == False):
#             pose_target = geometry_msgs.msg.Pose()
#             # Fixed object orientation
#             pose_target.orientation.w = 0.0
#             pose_target.orientation.x = 0.0
#             pose_target.orientation.y = -0.972
#             pose_target.orientation.z = 0.234
#             # Target object position
#             pose_target.position.x = coke[0]
#             pose_target.position.y = coke[1] - 0.1836
#             pose_target.position.z = coke[2] + 0.1
#             arm_group.set_pose_target(pose_target)
#             arm_group.set_goal_tolerance(0.01)
#             # Check if planning was successfull              
#             plan4 = arm_group.go()
#             # print the cordinates of object
#             print("x = {},y = {},z = {},plan = {}".format(coke[0], coke[1], coke[2], plan4))

#         plan1 = False
#         while(plan1 == False):
#             hand_group.set_named_target("close_coke")
#             plan1 = hand_group.go()

#         plan1 = False
#         while(plan1 == False):
#             arm_group.set_named_target("test")
#             plan1 = arm_group.go()

#     else :
#         plan1 = False
#         while(plan1 == False):
#             arm_group.set_named_target("test")
#             plan1 = arm_group.go()
#         t = TransformListener()
#         position = {'x': 11.302280 , 'y' : -0.975208}
#         quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 0.6985381175068214, 'r4' : 0.7155728462008786}
#         frequency = 60

#         # Print Cordinates to Console
#         rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
#         # Bot reached destination or not
#         result = navigator.goto(position, quaternion, frequency)
#         handle_result(result, position)

#         os.system('rosservice call /move_base/clear_costmaps "{}"')
#         arm_group.set_named_target("photo")
#         plan1 = arm_group.go()
#         t.waitForTransform("/ebot_base", "/object_145", rospy.Time(), rospy.Duration(4.0))
#         (coke,rotation) = t.lookupTransform("/ebot_base", "/object_145", rospy.Time())
#         plan4 = False
#         while(plan4 == False):
#             pose_target = geometry_msgs.msg.Pose()
#             # Fixed object orientation
#             pose_target.orientation.w = 0.0
#             pose_target.orientation.x = 0.0
#             pose_target.orientation.y = -0.972
#             pose_target.orientation.z = 0.234
#             # Target object position
#             pose_target.position.x = coke[0]
#             pose_target.position.y = coke[1] - 0.4
#             pose_target.position.z = coke[2] + 0.2
#             arm_group.set_pose_target(pose_target)
#             arm_group.set_goal_tolerance(0.01)
#             # Check if planning was successfull              
#             plan4 = arm_group.go()
#             # print the cordinates of object
#             print("x = {},y = {},z = {},plan = {}".format(coke[0], coke[1], coke[2], plan4))

#         plan4 = False
#         while(plan4 == False):
#             pose_target = geometry_msgs.msg.Pose()
#             # Fixed object orientation
#             pose_target.orientation.w = 0.0
#             pose_target.orientation.x = 0.0
#             pose_target.orientation.y = -0.972
#             pose_target.orientation.z = 0.234
#             # Target object position
#             pose_target.position.x = coke[0]
#             pose_target.position.y = coke[1] - 0.1836
#             pose_target.position.z = coke[2] + 0.1
#             arm_group.set_pose_target(pose_target)
#             arm_group.set_goal_tolerance(0.01)
#             # Check if planning was successfull              
#             plan4 = arm_group.go()
#             # print the cordinates of object
#             print("x = {},y = {},z = {},plan = {}".format(coke[0], coke[1], coke[2], plan4))

#         plan1 = False
#         while(plan1 == False):
#             hand_group.set_named_target("close_coke")
#             plan1 = hand_group.go()

#         plan4 = False
#         while(plan4 == False):
#             pose_target = geometry_msgs.msg.Pose()
#             # Fixed object orientation
#             pose_target.orientation.w = 0.0
#             pose_target.orientation.x = 0.0
#             pose_target.orientation.y = -0.972
#             pose_target.orientation.z = 0.234
#             # Target object position
#             pose_target.position.x = coke[0]
#             pose_target.position.y = coke[1] - 0.4
#             pose_target.position.z = coke[2] - 0.1
#             arm_group.set_pose_target(pose_target)
#             arm_group.set_goal_tolerance(0.01)
#             # Check if planning was successfull              
#             plan4 = arm_group.go()
#             # print the cordinates of object
#             print("x = {},y = {},z = {},plan = {}".format(coke[0], coke[1], coke[2], plan4))
        
#         # plan1 = False
#         # while(plan1 == False):
#         #     arm_group.set_named_target("travel1")
#         #     plan1 = arm_group.go()

#         plan1 = False
#         while(plan1 == False):
#             arm_group.set_named_target("test")
#             plan1 = arm_group.go()

# ##################################################################################
    
    
    
#     # attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
#     # attach_srv.wait_for_service()
#     # rospy.loginfo('Applying brakes to ebot')
#     # req = AttachRequest()
#     # req.model_name_1 = 'ebot'
#     # req.link_name_1 = 'ebot_base'
#     # req.model_name_2 = 'ground_plane'
#     # req.link_name_2 = 'link'
#     # attach_srv.call(req)

    
#     # rospy.sleep(2)
    
#     # attach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
#     # attach_srv.wait_for_service()
#     # rospy.loginfo('removing brakes to ebot')
#     # req = AttachRequest()
#     # req.model_name_1 = 'ebot'
#     # req.link_name_1 = 'ebot_base'
#     # req.model_name_2 = 'ground_plane'
#     # req.link_name_2 = 'link'
#     # attach_srv.call(req)
#     ###############################################################################

#                                  # PICK OBJECT

#     ###############################################################################

#     position = {'x': 13.01, 'y' :-0.870855}
#     quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 0.7023450278590759, 'r4' : 0.7118366820006073}
#     frequency = 60

#     # Print Cordinates to Console
#     rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
#     # Bot reached destination or not
#     result = navigator.goto(position, quaternion, frequency)
#     handle_result(result, position)

#     os.system('rosservice call /move_base/clear_costmaps "{}"')

#     ######    Moving to drop BOX 
#     position = {'x': 6.851449, 'y' : 2.520540}
#     quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 0.999988965231997, 'r4' : 0.004697809515074247}
#     frequency = 60

#     # Print Cordinates to Console
#     rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
#     # Bot reached destination or not
#     result = navigator.goto(position, quaternion, frequency)
#     handle_result(result, position)
#     os.system('rosservice call /move_base/clear_costmaps "{}"')

#     plan1 = False
#     while(plan1 == False):
#         arm_group.set_named_target("drop_left")
#         plan1 = arm_group.go()
    
#     plan1 = False
#     while(plan1 == False):
#         hand_group.set_named_target("open")
#         plan1 = hand_group.go()
    
#     plan1 = False
#     while(plan1 == False):
#         arm_group.set_named_target("travel1")
#         plan1 = arm_group.go()

#     plan1 = False
#     while(plan1 == False):
#         arm_group.set_named_target("test")
#         plan1 = arm_group.go()

#     position = {'x': 6.951449, 'y' : 2.520540}
#     quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 0.004697809515074247, 'r4' : 0.999988965231997}
#     frequency = 60

#     # Print Cordinates to Console
#     rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
#     # Bot reached destination or not
#     result = navigator.goto(position, quaternion, frequency)
#     handle_result(result, position)
#     os.system('rosservice call /move_base/clear_costmaps "{}"')

# ####################################################################################################

#     position = {'x': 7.875654, 'y' : 2.361752}
#     quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 0.010596801666939246, 'r4' : 0.9999438523209349}
#     frequency = 60

#     # Print Cordinates to Console
#     rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
#     # Bot reached destination or not
#     result = navigator.goto(position, quaternion, frequency)
#     handle_result(result, position)
#     os.system('rosservice call /move_base/clear_costmaps "{}"')

#     plan1 = False
#     while(plan1 == False):
#         arm_group.set_named_target("photo")
#         plan1 = arm_group.go()
#     t = TransformListener()
#     t.waitForTransform("/ebot_base", "/object_149", rospy.Time(), rospy.Duration(4.0))
#     (glue,rotation) = t.lookupTransform("/ebot_base", "/object_149", rospy.Time())

#     plan4 = False
#     while(plan4 == False):
#         pose_target = geometry_msgs.msg.Pose()
#         # Fixed object orientation
#         pose_target.orientation.w = 0.0
#         pose_target.orientation.x = 0.0
#         pose_target.orientation.y = -0.972
#         pose_target.orientation.z = 0.234
#         # Target object position
#         pose_target.position.x = glue[0] - 0.008
#         pose_target.position.y = glue[1] - 0.4
#         pose_target.position.z = glue[2] + 0.2
#         arm_group.set_pose_target(pose_target)
#         arm_group.set_goal_tolerance(0.01)
#         # Check if planning was successfull              
#         plan4 = arm_group.go()
#         # print the cordinates of object
#         print("x = {},y = {},z = {},plan = {}".format(glue[0], glue[1], glue[2], plan4))

#     plan4 = False
#     while(plan4 == False):
#         pose_target = geometry_msgs.msg.Pose()
#         # Fixed object orientation
#         pose_target.orientation.w = 0.0
#         pose_target.orientation.x = 0.0
#         pose_target.orientation.y = -0.972
#         pose_target.orientation.z = 0.234
#         # Target object position
#         pose_target.position.x = glue[0] - 0.008
#         pose_target.position.y = glue[1] - 0.195
#         pose_target.position.z = glue[2] + 0.1
#         arm_group.set_pose_target(pose_target)
#         arm_group.set_goal_tolerance(0.01)
#         # Check if planning was successfull              
#         plan4 = arm_group.go()
#         # print the cordinates of object
#         print("x = {},y = {},z = {},plan = {}".format(glue[0], glue[1], glue[2], plan4))

#     plan1 = False
#     while(plan1 == False):
#         hand_group.set_named_target("close_glue")
#         plan1 = hand_group.go()

#     plan4 = False
#     while(plan4 == False):
#         pose_target = geometry_msgs.msg.Pose()
#         # Fixed object orientation
#         pose_target.orientation.w = 0.0
#         pose_target.orientation.x = 0.0
#         pose_target.orientation.y = -0.972
#         pose_target.orientation.z = 0.234
#         # Target object position
#         pose_target.position.x = glue[0] - 0.008
#         pose_target.position.y = glue[1] - 0.4
#         pose_target.position.z = glue[2] + 0.2
#         arm_group.set_pose_target(pose_target)
#         arm_group.set_goal_tolerance(0.01)
#         # Check if planning was successfull              
#         plan4 = arm_group.go()
#         # print the cordinates of object
#         print("x = {},y = {},z = {},plan = {}".format(glue[0], glue[1], glue[2], plan4))

#     plan1 = False
#     while(plan1 == False):
#         arm_group.set_named_target("travel1")
#         plan1 = arm_group.go()

#     plan1 = False
#     while(plan1 == False):
#         arm_group.set_named_target("test")
#         plan1 = arm_group.go()

    
#     # ###############################################################################

#     #                     #  DROP OBJECT

#     # ###############################################################################

#     position = {'x': 7.00, 'y' : 2.681765}
#     quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 0.0, 'r4' : 1.0}
#     frequency = 60

#     # Print Cordinates to Console
#     rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
#     # Bot reached destination or not
#     result = navigator.goto(position, quaternion, frequency)
#     handle_result(result, position)
#     os.system('rosservice call /move_base/clear_costmaps "{}"')

#     plan1 = False
#     while(plan1 == False):
#         arm_group.set_named_target("drop_left")
#         plan1 = arm_group.go()
    
#     plan1 = False
#     while(plan1 == False):
#         hand_group.set_named_target("open")
#         plan1 = hand_group.go()

#######################################################################################################################
    # ################################################################################

    #                 #  PICK OBJECT

    # ################################################################################
    
    # position = {'x': 10.9, 'y' : 9.43}
    # quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : 0.7071045443232221, 'r4' : 0.7071090180427968}
    # frequency = 60

    # # Print Cordinates to Console
    # rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
    # # Bot reached destination or not
    # result = navigator.goto(position, quaternion, frequency)
    # handle_result(result, position)


    # position = {'x': 10.9, 'y' : 9.43}
    # quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : -0.703534947165817, 'r4' : 0.7106606631271996}
    # frequency = 60

    # # Print Cordinates to Console
    # rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
    # # Bot reached destination or not
    # result = navigator.goto(position, quaternion, frequency)
    # handle_result(result, position)
    # os.system('rosservice call /move_base/clear_costmaps "{}"')


    # position = {'x': 26.159948, 'y' : -2.820398}
    # quaternion = {'r1' : 0.0, 'r2' : 0.0, 'r3' : -0.8895252423753343, 'r4' : 0.4568860286516789}
    # frequency = 60

    # # Print Cordinates to Console
    # rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
    # # Bot reached destination or not
    # result = navigator.goto(position, quaternion, frequency)
    # handle_result(result, position)

    plan1 = False
    while(plan1 == False):
        arm_group.set_named_target("photo")
        plan1 = arm_group.go()
    t = TransformListener()
    t.waitForTransform("/ebot_base", "/object_155", rospy.Time(), rospy.Duration(4.0))
    (glue,rotation) = t.lookupTransform("/ebot_base", "/object_155", rospy.Time())

    plan4 = False
    while(plan4 == False):
        pose_target = geometry_msgs.msg.Pose()
        # Fixed object orientation
        pose_target.orientation.w = -0.033
        pose_target.orientation.x = 0.206
        pose_target.orientation.y =  0.940
        pose_target.orientation.z = -0.269
        # Target object position
        pose_target.position.x = glue[0] - 0.13
        pose_target.position.y = glue[1] - 0.2
        pose_target.position.z = glue[2] + 0.2
        arm_group.set_pose_target(pose_target)
        arm_group.set_goal_tolerance(0.01)
        # Check if planning was successfull              
        plan4 = arm_group.go()
        # print the cordinates of object
        print("x = {},y = {},z = {},plan = {}".format(glue[0], glue[1], glue[2], plan4))

    plan4 = False
    while(plan4 == False):
        pose_target = geometry_msgs.msg.Pose()
        # Fixed object orientation
        pose_target.orientation.w = -0.033
        pose_target.orientation.x = 0.206
        pose_target.orientation.y =  0.940
        pose_target.orientation.z = -0.269
        # Target object position
        pose_target.position.x = glue[0] - 0.13
        pose_target.position.y = glue[1] - 0.18
        pose_target.position.z = glue[2] + 0.1
        arm_group.set_pose_target(pose_target)
        arm_group.set_goal_tolerance(0.01)
        # Check if planning was successfull              
        plan4 = arm_group.go()
        # print the cordinates of object
        print("x = {},y = {},z = {},plan = {}".format(glue[0], glue[1], glue[2], plan4))
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
