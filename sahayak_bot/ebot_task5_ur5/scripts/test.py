#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import sys
import moveit_commander
import geometry_msgs.msg
from actionlib_msgs.msg import *
from trajectory_msgs.msg import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

if __name__ == '__main__':   
        rospy.sleep(5)   
        print("start") 
        rospy.init_node('task4_gazebo_sim')
        attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        attach_srv.wait_for_service()
        rospy.loginfo('Applying brakes to ebot')
        req = AttachRequest()
        req.model_name_1 = 'ebot'
        req.link_name_1 = 'ebot_base'
        req.model_name_2 = 'ground_plane'
        req.link_name_2 = 'link'
        attach_srv.call(req)
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        hand_group = moveit_commander.MoveGroupCommander("grip_planning_group")
        arm_group = moveit_commander.MoveGroupCommander("arm_planning_group")
        plan1 = False
        while(plan1==False):
            arm_group.set_named_target("photo")
            plan1 = arm_group.go()