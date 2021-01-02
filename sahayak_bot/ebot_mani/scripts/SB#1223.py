#! /usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('moveit_test', anonymous=True)
robot = moveit_commander.RobotCommander()
hand_group = moveit_commander.MoveGroupCommander("gripper_planning_group")
arm_group = moveit_commander.MoveGroupCommander("ur5_1_planning_group")

# Orientation and Position for blue box
blue_box_orientation = {'w': 0.5, 'x' : -0.5, 'y' : 0.5, 'z' : -0.5}
blue_box_position = {'x' : 0.000000, 'y' : -0.710000, 'z' : 1.105000}
# Orientation and Position for red box
red_box_orientation = {'w': 0.5, 'x' : -0.5, 'y' : 0.5, 'z' : -0.5}
red_box_position = {'x' : 0.000000, 'y' : 0.710000, 'z' : 1.105000}

def arm_straight_position():
    # Put the arm in the Straight position
    arm_group.set_named_target("Straightup")
    plan1 = arm_group.go()

def gripper_open():
    # griper open position
    hand_group.set_named_target("Open")
    plan2 = hand_group.go()

def move_arm_to_cordinates(orientation, position):
    # cordinates and orientation of target
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = orientation['w']  
    pose_target.orientation.x = orientation['x']
    pose_target.orientation.y = orientation['y']
    pose_target.orientation.z = orientation['z']
    pose_target.position.x = position['x']
    pose_target.position.y = position['y']
    pose_target.position.z = position['z']
    arm_group.set_pose_target(pose_target)
    plan4 = arm_group.go()


def gripper_close(close_pos):
    # Close gripper to grab
    hand_group.set_named_target(close_pos)
    plan2 = hand_group.go()

# Main program

def arm_driver():
    # Bring arm and gripper in initial postion
    arm_straight_position()
    gripper_open()

   # Orientation and position for Object 1
    orientation = {'w': 0.653, 'x' : -0.653, 'y' : 0.272, 'z' : -0.272}
    position = {'x' : 0.560000, 'y' : -0.00000, 'z' : 0.942496}
    # Move arm to Object 1
    move_arm_to_cordinates(orientation, position)
    # Bring arm down to close gripper to grab object
    orientation = {'w': 0.653, 'x' : -0.653, 'y' : 0.272, 'z' : -0.272}
    position = {'x' : 0.560000, 'y' : -0.00000, 'z' : 0.862496}
    move_arm_to_cordinates(orientation, position)
    gripper_close("Close3")

    # Cordinates and orientation of red box
    move_arm_to_cordinates(red_box_orientation, red_box_position)
    # Drop Object in the box
    gripper_open()
    ## Object 1 done

    # Orientation and position for Object 2
    orientation = {'w': 0.261, 'x' : -0.261, 'y' : 0.657, 'z' : -0.657}
    position = {'x' : 0.460000, 'y' : 0.220000, 'z' : 0.922496}
    # Move arm to Object 2
    move_arm_to_cordinates(orientation,position)
    # Bring arm down to close gripper to grab object
    orientation = {'w': 0.261, 'x' : -0.261, 'y' : 0.657, 'z' : -0.657}
    position = {'x' : 0.460000, 'y' : 0.220000, 'z' : 0.832496}
    move_arm_to_cordinates(orientation,position)
    gripper_close("Close")

    # Cordinates and orientation of blue box
    move_arm_to_cordinates(blue_box_orientation,blue_box_position)
    # Drop Object in the box
    gripper_open()
    ##Object 2 Done

    # Orientation and position for Object 3
    orientation = {'w': 0.707, 'x' : -0.707, 'y' : 0.0, 'z' : 0.0}
    position = {'x' : 0.540000, 'y' : -0.240000, 'z' : 0.942496}
    # Move arm to Object 3
    move_arm_to_cordinates(orientation, position)
    # Bring arm down to close gripper to grab object
    orientation = {'w': 0.707, 'x' : -0.707, 'y' : 0.0, 'z' : 0.0}
    position = {'x' : 0.540000, 'y' : -0.240000, 'z' : 0.88}
    move_arm_to_cordinates(orientation, position)
    gripper_close("Close2")

    # Cordinates and orientation of blue box
    move_arm_to_cordinates(blue_box_orientation, blue_box_position)
    # Drop Object in the box
    gripper_open()
    ## Object 3 done


    rospy.sleep(2)
    moveit_commander.roscpp_shutdown()

# Python Main
if __name__ == '__main__':
    try:
        arm_driver()
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C Quit")
