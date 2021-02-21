#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry
pantryToBePrinted  = True
meetingRoomToBePrinted  = True
conferenceRoomToBePrinted  = True
researchLabToBePrinted  = True
storeRoomToBePrinted  = True

def func_callback_topic_my_topic(myMsg):
    x = myMsg.pose.pose.position.x
    y = myMsg.pose.pose.position.y
    global pantryToBePrinted
    global meetingRoomToBePrinted
    global conferenceRoomToBePrinted
    global researchLabToBePrinted
    global storeRoomToBePrinted

    if(pantryToBePrinted and 12.608207 < x < 13.268829 and -0.332767 < y < 0):
        # print('x is ',x)
        # print('y is ', y)
        print('Pantry Reached')
        pantryToBePrinted = False
    
    elif(meetingRoomToBePrinted and 8.253483 < x < 9.092000 and 2.070465 < y < 2.459742):
        # print('x is ',x)
        # print('y is ', y)
        print('Meeting Room Reached')
        meetingRoomToBePrinted = False

    elif(conferenceRoomToBePrinted and 4.802292 < x < 5.651354 and -0.332767 < y < 0):
        # print('x is ',x)
        # print('y is ', y)
        print('Conference Room Reached')
        conferenceRoomToBePrinted = False
    
    elif(researchLabToBePrinted and 10.025547 < x < 11.459243 and 7.34520 < y < 7.8):
        # print('x is ',x)
        # print('y is ', y)
        print('Research Lab Reached')
        researchLabToBePrinted = False
    
    elif(storeRoomToBePrinted and 24.975998 < x < 25.990601 and -3.171468 < y < -1.281662):
        # print('x is ',x)
        # print('y is ', y)
        print('Store Room Reached')
        storeRoomToBePrinted = False

def main():

    # 1. Initialize the Subscriber Node.
    rospy.init_node('node_myMsg_listener', anonymous=True)

    # 2. Subscribe to the desired topic and attach a Callback Funtion to it.
    rospy.Subscriber("/odom", Odometry, func_callback_topic_my_topic)

    # 3. spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


# Python Main
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass





####################### Code for terminal
# import os
# import rospy    
# os.system("gnome-terminal --tab -- roslaunch my_object_recognition_pkg start_find_object_3d_session.launch")      
# rospy.sleep(5)
# # os.system("gnome-terminal --tab -- roslaunch my_object_recognition_pkg start_find_object_3d_session.launch")      
# # rospy.sleep(5)
# # os.system("kill `ps -e | grep -i gnome-terminal | cut -f 1 -d \" \"`")
# os.system("pkill gnome-terminal")  
# os.system("gnome-terminal --tab -- roslaunch my_object_recognition_pkg start_find_object_3d_session.launch")      
# rospy.sleep(5)
# os.system("pkill gnome-terminal")  

# os.system("gnome-terminal --tab -- roslaunch my_object_recognition_pkg start_find_object_3d_session.launch")      
# rospy.sleep(5)
# os.system("pkill gnome-terminal")  
