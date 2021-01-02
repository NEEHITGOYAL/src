#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
is_complete = False

def pose_callback(msg):
    # print(msg.theta))
    if(msg.theta) > -0.1 and msg.theta < 0:
        print("moving in circle")
        global is_complete
        is_complete = True

        #set linear and angular velocity to be 0 and stop the turtle
        move_turtle(0, 0)

def move_turtle(x_lin_vel,z_ang_vel):
    # Create a handle to publish messages to a topic.
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    # set the linear velocity
    vel_msg.linear.x = x_lin_vel
    # set the angular velocity
    vel_msg.angular.z = z_ang_vel

    # publish the values of vel_msg
    velocity_publisher.publish(vel_msg)

def main():
    
    #Initializes the ROS node for the process.
    rospy.init_node('node_turtle_revolve', anonymous=True)

    # 1 Hz : Loop will its best to run 1 time in 1 second
    var_loop_rate = rospy.Rate(1) 

    #loop
    while not rospy.is_shutdown():

        #check if complete
        if(is_complete == True):
            break

        #Create a handle to Subscribe messages from a topic.
        rospy.Subscriber("/turtle1/pose", Pose, pose_callback)

        #set the linear and angular velocity to be 1
        move_turtle(1, 1)

        var_loop_rate.sleep()



# Python Main
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
