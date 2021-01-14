#! /usr/bin/env python
import sys
import os
import rospy
import moveit_commander
import geometry_msgs.msg
from tf import TransformListener
from object_msgs.msg import ObjectPose

global coke_target,battery_target,glue_target
coke_target=[0.0,0.0,0.0]
battery_target=[0.0,0.0,0.0]
glue_target=[0.0,0.0,0.0]

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


def control_loop():
    
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
    coke_target[1] = coke[1] - 0.1836
    coke_target[2] = coke[2] + 0.1
    # Move arm to grab coke
    armPlanner(coke_target)
    
    # Move gripper to close_coke pose
    gripperPose("close_coke")

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
    battery_target[0] = battery[0] - 0.005
    battery_target[1] = battery[1] - 0.40
    battery_target[2] = battery[2] + 0.09
    # Move arm in front of battery
    armPlanner(battery_target)

    # Tune cordinates
    battery_target[0] = battery[0] - 0.005
    battery_target[1] = battery[1] - 0.184
    battery_target[2] = battery[2] + 0.09
    # Move arm to grab battery
    armPlanner(battery_target)

    # Move gripper to close_battery pose
    gripperPose("close_battery")

    # Tune cordinates
    battery_target[0] = battery[0] - 0.005
    battery_target[1] = battery[1] - 0.40
    battery_target[2] = battery[2] + 0.2
    # Move arm back
    armPlanner(battery_target)

    # Move arm to drop pose
    armPose("drop")

    # Move gripper to open pose
    gripperPose("open")

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

    # Tune cordinates
    glue_target[0] = glue[0] - 0.008
    glue_target[1] = glue[1] - 0.195
    glue_target[2] = glue[2] + 0.2
    # Move arm back
    armPlanner(glue_target)

    # Move arm to drop pose
    armPose("drop")

    # Move gripper to open pose
    gripperPose("open")        

    rospy.sleep(5)
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass