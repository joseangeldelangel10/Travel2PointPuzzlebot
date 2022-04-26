#!/usr/bin/env python
# coding=utf-8
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float64
import cv2 as cv
import cv_bridge



class ImageProcessor():
    def __init__(self):
        #Inicializamos el nodo
        rospy.init_node("ImageProcessor")       
        #Creamos los susbcribers
        self.imageSubscriber = rospy.Subscriber("/camera/image_raw",Image,self.on_image_callback)        
        self.bridge = cv_bridge.CvBridge()        
        #Creamos los publishers
        #self.pub_odom = rospy.Publisher("/pose2d", Pose2D, queue_size=1)        
        #Iniciamos el voctor de posición en cero
        self.image = None
        self.rate = rospy.Rate(0.1)
    
    def on_image_callback(self, image):
    	self.image = image

    def main(self):
    	while not rospy.is_shutdown():
    		if (self.image != None):
    			self.rate.sleep()
    			cv_image = self.bridge.imgmsg_to_cv2(self.image, desired_encoding="bgr8")
    			cv.imshow("original img", cv_image)
    			print("image converted succesfully")
    			cv_gray_image = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
    			cv.imshow("gray img", cv_gray_image)

if __name__ == "__main__":
	ip = ImageProcessor()
	ip.main()




