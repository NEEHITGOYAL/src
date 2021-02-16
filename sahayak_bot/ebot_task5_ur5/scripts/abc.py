import numpy as np
import argparse
import cv2

image = cv2.imread("/home/neehit/catkin_ws/src/sahayak_bot/my_object_recognition_pkg/saved_pictures/screenshots/210216170529086.jpg")
boundaries = [
	([17, 15, 100], [50, 56, 200]),
	([86, 31, 4], [220, 88, 50]),
	([25, 146, 190], [62, 174, 250]),
	([103, 86, 65], [145, 133, 128])
]
for (lower, upper) in boundaries:
	lower = np.array(lower, dtype = "uint8")
	upper = np.array(upper, dtype = "uint8")
	if cv2.inRange(image, lower, upper).any():
    	    image = cv2.putText(image, 'OpenCV', (50, 50) ,cv2.FONT_HERSHEY_SIMPLEX ,1,(17, 15, 100),2, cv2.LINE_AA) 
	# show the images
    cv2.imshow("images",image)
    cv2.waitKey(0)