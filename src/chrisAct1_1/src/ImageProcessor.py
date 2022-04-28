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
        self.pub_gray = rospy.Publisher("/processedImage/gray", Image, queue_size=1)        
        #Iniciamos el voctor de posición en cero
        self.image = None
        self.rate = rospy.Rate(0.1)

        self.cv_image = np.zeros((300,300, 300))
        self.cv_gray_image = np.zeros((300,280))
        self.gray_img_msg = Image()

    
    def on_image_callback(self, image):
        self.image = image

    def main(self):
        while not rospy.is_shutdown():
            if (self.image != None):
                self.rate.sleep()
                self.cv_image = self.bridge.imgmsg_to_cv2(self.image, desired_encoding="passthrough")
                #cv.imshow("original img", cv_image)
                print("image converted succesfully")
                self.cv_gray_image = cv.Canny(self.cv_image, 50, 200)
                #cv_gray_image = None
                #print(type(cv_gray_image))
                #cv.imshow("gray img", cv_gray_image)
                self.gray_img_msg = self.bridge.cv2_to_imgmsg(self.cv_gray_image, encoding = "passthrough")
                self.pub_gray.publish(self.gray_img_msg)


if __name__ == "__main__":
    ip = ImageProcessor()
    ip.main()




