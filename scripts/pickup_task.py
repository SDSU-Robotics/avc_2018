#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('avc_2018')
import sys
import rospy
import cv2
import matplotlib.pyplot as plt
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

SHOW_INTERMEDIATE = False

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("raw_image",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    #(rows,cols,channels) = cv_image.shape
    
    #if cols > 60 and rows > 60 :
     # cv2.circle(cv_image, (700,300), 250, 255)

    #OpenCV uses BGR so we need band 3 for the red band
    gray = cv_image[:,:,2]
    #Blur the image to remove noise. GaussainBlur(src, ksize, sigmaX)
    blurred = cv2.GaussianBlur(gray, (11,11), 0)

    #set threshold
    thresh = cv2.threshold(blurred, 220, 255, cv2.THRESH_TOZERO)[1]

    if SHOW_INTERMEDIATE:
      i = 0
      plt.figure(100*(i+1)+1)

      plt.imshow(cv_image[...,::-1])
      plt.title('Original Image')

      plt.figure(100*(i+1)+2)
      plt.imshow(gray, cmap='gray')
      plt.title('Red Band of Image')

      plt.figure(100*(i+1)+3)
      plt.imshow(blurred, cmap='gray')
      plt.title('Blurred Image')

      plt.figure(100*(i+1)+4)
      plt.imshow(thresh, cmap='gray')
      plt.title('Thresholded Image')

    #Find contours in the image
    contours = cv2.findContours(thresh.copy(), #Need to copy the image
                            cv2.RETR_EXTERNAL, #This is the correct mode
                            cv2.CHAIN_APPROX_SIMPLE) #Approximation method

    contours = contours[1]

    #Find the largest contour since that is probably what we want
    c = max(contours, key=cv2.contourArea)

    M = cv2.moments(c)
    try:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        #draw the contour and center of the shape on the image
        cv2.drawContours(cv_image, [c], -1, (0, 255, 0), 2)
        cv2.circle(cv_image, (cX, cY), 7, (0, 255, 0), -1)
        cv2.putText(cv_image, "center", (cX - 20, cY - 20),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        #show the image
#        plt.imshow(cv_image[...,::-1])
#        plt.show()

    except ZeroDivisionError:
      print('Zero division error, skipping')

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)