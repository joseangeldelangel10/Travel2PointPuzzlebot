#!/usr/bin/env python
# coding=utf-8
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float64, Bool
import cv2 as cv
import cv_bridge



class ImageProcessor():
    def __init__(self):
        #Inicializamos el nodo
        rospy.init_node("RedCiclesDetector")       
        #susbcribers
        self.imageSubscriber = rospy.Subscriber("/video_source/raw",Image,self.on_image_callback)        
        self.bridge = cv_bridge.CvBridge()        
        #publishers
        self.pub_red = rospy.Publisher("/processedImage/detectedObjects/redCircle", Image, queue_size=1)
        self.pub_red_img = rospy.Publisher("/processedImage/detectedObjects/redCircleImage", Image, queue_size=1)        
        #Iniciamos el voctor de posición en cero
        self.image = None
        self.rate = rospy.Rate(0.1)

        self.cv_image = np.zeros((300,300, 300))
        self.red1 = np.zeros((300,300, 300))
        self.red2 = np.zeros((300,300, 300))
        self.processedImage = np.zeros((300,280,300))
        self.outImage = np.zeros((300,280,300))
        self.processed_image_msg = Image()

    
    def on_image_callback(self, image):
    	self.image = image

    def main(self):
    	while not rospy.is_shutdown():
    		if (self.image != None):
    			self.rate.sleep()
    			self.cv_image = self.bridge.imgmsg_to_cv2(self.image, desired_encoding="passthrough")
    			
    			#print("image converted succesfully")
    			

                self.processedImage = cv.cvtColor(self.cv_image, cv.COLOR_BGR2HSV)
    
                self.red1 = cv.inRange(self.procesedImage,(170,70,50),(180,255,255))
                self.red2 = cv.inRange(self.procesedImage,(0,70,50),(10,255,255))

                self.proccesedImage = cv.bitwise_or(self.red1,self.red2)
                self.processedImage = cv.morphologyEx(self.processedImage, cv.MORPH_CLOSE, smaller_kernel)
                self.processedImage = cv.dilate(self.processedImage,smaller_kernel,iterations = 1)    
                self.processedImage = cv.bitwise_not(self.processedImage)


                (cols, rows) = self.processedImage.shape

                blobDetectorParams = cv.SimpleBlobDetector_Params()
                blobDetectorParams.filterByCircularity = True    
                blobDetectorParams.minCircularity = 0.81
                blobDetectorParams.maxCircularity = 1.0
                blobDetectorParams.filterByArea = True
                blobDetectorParams.minArea = (rows*cols)*(1/50)
                blobDetectorParams.maxArea = (rows*cols)*(3/4)
                
                ver = (cv.__version__).split('.')
                if int(ver[0]) < 3 :
                    detector = cv.SimpleBlobDetector(blobDetectorParams)
                else : 
                    detector = cv.SimpleBlobDetector_create(blobDetectorParams)
                
                keypoints = detector.detect(self.processedImage)

                self.outImage = cv.drawKeypoints(self.processedImage, keypoints, 0, (0,0,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

                result = Bool()
                if(keypoints != 0):                    
                    result.date = True
                    self.pub_red.publish(result)
                else:
                    result.date = False
                    self.pub_red.publish(result)

                self.processed_image_msg = self.bridge.cv2_to_imgmsg(self.outImage, encoding = "passthrough")
                self.pub_red_img.publish(self.processed_image_msg)


if __name__ == "__main__":
	ip = ImageProcessor()
	ip.main()




