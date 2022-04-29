#!/usr/bin/env python
# coding=utf-8
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float64
import cv2 as cv
import cv_bridge
import time


class Photographer():
    def __init__(self):
        #Inicializamos el nodo
        rospy.init_node("Photographer")       
        #Creamos los susbcribers
        self.imageSubscriber = rospy.Subscriber("/camera/image_raw",Image,self.on_image_callback)        
        self.bridge = cv_bridge.CvBridge()        
        
        self.new_images_dir = "./takenPhotos/"        
        self.image_id = str(time.time())
        self.image = None
        self.rate = rospy.Rate(1)

        self.iteration = 0
        self.cv_image = np.zeros((300,300, 300))

    
    def on_image_callback(self, image):
    	self.image = image

    def main(self):
    	while not rospy.is_shutdown():
    		if (self.image != None):
    			self.rate.sleep()
    			if(iteration < 10):
                    print("taking photo in {counter}".format(counter=10-iteration))
    			elif(iteration == 10):                    
                    self.cv_image = self.bridge.imgmsg_to_cv2(self.image, desired_encoding="passthrough")                   
                    cv.imwrite(self.new_images_dir + self.image_id + ".jpg", self.cv_image)
                    print("image saved succesfully")

if __name__ == "__main__":
	ip = Photographer()
	ip.main()




