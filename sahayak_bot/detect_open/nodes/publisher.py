#!/usr/bin/env python
import roslib; roslib.load_manifest('detect_open')
import rospy
import sys, os ,cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from object_msgs.msg import Tracker
from sensor_msgs.msg import Image
class cvBridgeDemo():
    def __init__(self):
        self.node_name = "cv_bridge_demo"
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.cleanup)
        self.cv_window_name = self.node_name
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw2", Image, self.image_callback)
        self.image_pub = rospy.Publisher('cxy', Tracker, queue_size=1)
        rospy.loginfo("Waiting for image topics...")
    def image_callback(self, ros_image):
         try:
             frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
         except CvBridgeError:
            print("Error") 
         new_path="/home/narayan/catkin_ws/src/sahayak_bot/detect_open/obj"
         threshold=0.80
         main_image=frame
         test_image= cv2.cvtColor(main_image, cv2.COLOR_BGR2GRAY)
         for file_name in os.listdir(new_path):
             sub_dir_path = new_path + '/' + file_name
             if (os.path.isdir(sub_dir_path)):
                 for image_name in os.listdir(sub_dir_path):
                    image = cv2.imread(sub_dir_path+"/"+image_name,0)
                    (tH, tW) = image.shape[:2]
                    result = cv2.matchTemplate(test_image, image,cv2.TM_CCOEFF_NORMED)
                    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
                    top_left = max_loc
                    bottom_right = (top_left[0] + tW, top_left[1] + tH)
                    if np.amax(result)>threshold:
                        cv2.rectangle(main_image,top_left, bottom_right,255, 2)
                        cv2.putText(main_image ,file_name,top_left,cv2.FONT_HERSHEY_SIMPLEX,0.75,(255,0,0),2,cv2.LINE_AA)
                        print("{} detected".format(file_name))
                        break
         cv2.imshow('Result',main_image)
         self.keystroke = cv2.waitKey(5)
         if 32 <= self.keystroke and self.keystroke < 128:
             cc = chr(self.keystroke).lower()
             if cc == 'q':
                 # The user has press the q key, so exit
                 rospy.signal_shutdown("User hit q key to quit.")
    def cleanup(self):
        print ("Shutting down vision node.")
        cv2.destroyAllWindows()   
def main(args):       
    try:
        cvBridgeDemo()
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down vision node.")
        cv2.DestroyAllWindows()
if __name__ == '__main__':
    main(sys.argv)