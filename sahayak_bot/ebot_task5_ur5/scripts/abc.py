#!/usr/bin/env python
import rospy
import os

if __name__ == '__main__':
    rospy.sleep(4)
    try:
        os.system("gnome-terminal -- roslaunch my_object_recognition_pkg start_find_object_3d_session.launch")
        rospy.logerr('err')
        print('dsfdsffdgdfg')
        os.system("rosnode kill "+ '/find_object_3d')
        os.system("rosnode kill "+ '/tf_example')
        rospy.logerr('err2')
        os.system("gnome-terminal -- roslaunch my_object_recognition_pkg start_find_object_3d_session.launch")
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C Quit")