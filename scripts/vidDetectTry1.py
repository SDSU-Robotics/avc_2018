#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""

DetectRedSquare.py

Written by Calvin Kielas-Jensen on 8-28-2018



Last updated on 8-28-2018



This is meant to be a simple example of how to detect a red square in an image

with noise. The majority of this code came from

https://www.pyimagesearch.com/2016/02/01/opencv-center-of-contour/

"""



__author__ = 'Calvin Kielas-Jensen'

__version__ = '0.0.1'

__maintainer__ = 'SDSU Robotics Club'

#__email__ = 'todo@todo.edu'

#__status__ = 'Development'



import os

import cv2

import matplotlib.pyplot as plt



# Set this to True to see each step of the image processing

SHOW_INTERMEDIATE = False



ROOT_DIR = os.path.dirname(os.path.abspath(__file__))

IMAGE_DIR = os.path.join(ROOT_DIR, 'TestImages')



plt.close('all')



# Change to the image directory

os.chdir(IMAGE_DIR)



# Get the names of the images in the directory

imageNames = os.listdir(IMAGE_DIR)

imageNames = [i for i in imageNames if '.png' in i or '.jpg' in i]



for i, imageName in enumerate(imageNames):

    # read the image

    image = cv2.imread(imageName)

    # OpenCV uses BGR so we need band 3 for the red band

    gray = image[:,:,2]

    # Blur the image to remove noise. GaussianBlur(src, ksize, sigmaX)

    blurred = cv2.GaussianBlur(gray, (11,11), 0) 

    

#    thresh = cv2.adaptiveThreshold(blurred, # Source image

#                                   255, # Max value

#                                   cv2.ADAPTIVE_THRESH_GAUSSIAN_C, # method

#                                   cv2.THRESH_BINARY,# type

#                                   11, # Block size, must be > 3 and odd

#                                   2) # Some constant C, see man for more info

    

    # Threshold the image (I would prefer an adaptive threshold, but I

    # couldn't get the code above to work, it has to do with C I think)

    # May have to play with the threshold to get it to work properly

    thresh = cv2.threshold(blurred, 200, 255, cv2.THRESH_TOZERO)[1]

    

    if SHOW_INTERMEDIATE:

        plt.figure(100*(i+1)+1)

        # No idea why the ... thing works, but it converts from BGR to RGB

        # since OpenCV is weird and uses BGR images where everything else 

        # uses RGB.

        plt.imshow(image[...,::-1])

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

    

    # Find contours in the image (essentially full shapes, Google image

    # contours if you're curious)

    contours = cv2.findContours(thresh.copy(), # Need to copy the image

                                cv2.RETR_EXTERNAL, # This is the correct mode

                                cv2.CHAIN_APPROX_SIMPLE) # Approximation method

    

    # WARNING, CHANGE FOR OPENCV VERSION

    contours = contours[1] # For OpenCV 3

    #contours = contours[0] # For OpenCV 2.4

    

    # Find the largest contour which is probably what we are looking for.

    # A lot of other contours are noisy and probably not what we want.

    c = max(contours, key=cv2.contourArea)

    

    M = cv2.moments(c)

    try:

        cX = int(M["m10"] / M["m00"])

        cY = int(M["m01"] / M["m00"])

         

        # draw the contour and center of the shape on the image

        cv2.drawContours(image, [c], -1, (0, 255, 0), 2)

        cv2.circle(image, (cX, cY), 7, (0, 255, 0), -1)

        cv2.putText(image, "center", (cX - 20, cY - 20),

        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

         

        # show the image

#            cv2.imshow("Image", image)

#            cv2.waitKey(0)

        plt.figure(100*(i+1))

        plt.imshow(image[...,::-1])

        plt.show()



    except ZeroDivisionError:

        print('Zero division error, skipping')  