#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from colorama import Fore, Back, Style

# Declaring and initializing variables used to print only once
pantryToBePrinted  = True
meetingRoomToBePrinted  = True
conferenceRoomToBePrinted  = True
researchLabToBePrinted  = True
storeRoomToBePrinted  = True

# function to print with
def myPrint(string):
    print(Fore.GREEN + Back.WHITE + string + Style.RESET_ALL)

def func_callback_topic_my_topic(myMsg):

    # Storing x and y cordinates of bot
    x = myMsg.pose.pose.position.x
    y = myMsg.pose.pose.position.y
    global pantryToBePrinted
    global meetingRoomToBePrinted
    global conferenceRoomToBePrinted
    global researchLabToBePrinted
    global storeRoomToBePrinted

    # Check if bot has reached pantry for first time
    if(pantryToBePrinted and 12.608207 < x < 13.268829 and -0.332767 < y < 0):
        myPrint('Pantry Reached')
        pantryToBePrinted = False
    
    # Check if bot has reached Meeting Room for first time
    elif(meetingRoomToBePrinted and 8.253483 < x < 9.092000 and 2.070465 < y < 2.459742):
        myPrint('Meeting Room Reached')
        meetingRoomToBePrinted = False

    # Check if bot has reached Conference Room for first time
    elif(conferenceRoomToBePrinted and 4.802292 < x < 5.651354 and -0.332767 < y < 0):
        myPrint('Conference Room Reached')
        conferenceRoomToBePrinted = False
    
    # Check if bot has reached Research Lab for first time
    elif(researchLabToBePrinted and 10.025547 < x < 11.459243 and 7.34520 < y < 7.8):
        myPrint('Research Lab Reached')
        researchLabToBePrinted = False
    
    # Check if bot has reached Store Room for first time
    elif(storeRoomToBePrinted and 24.975998 < x < 25.990601 and -3.171468 < y < -1.281662):
        myPrint('Store Room Reached')
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
