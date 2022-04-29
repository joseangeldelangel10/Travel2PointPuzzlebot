#!/usr/bin/env python
# coding=utf-8
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float64, Bool
import cv2 as cv
import cv_bridge



class redCirclesDetector():
    def __init__(self):
        #Inicializamos el nodo
        rospy.init_node("RedCirclesDetector")       
        #susbcribers
        self.imageSubscriber = rospy.Subscriber("/video_source/raw",Image,self.on_image_callback)        
        self.bridge = cv_bridge.CvBridge()        
        #publishers
        self.pub_red = rospy.Publisher("/processedImage/detectedObjects/redCircle", Bool, queue_size=10)
        self.pub_red_img = rospy.Publisher("/processedImage/detectedObjects/redCircleImage", Image, queue_size=10)
        #self.pub_red_img1 = rospy.Publisher("/processedImage/detectedObjects/redCircleImage1", Image, queue_size=10)
        #self.pub_red_img2 = rospy.Publisher("/processedImage/detectedObjects/redCircleImage2", Image, queue_size=10)
        #self.pub_red_img3 = rospy.Publisher("/processedImage/detectedObjects/redCircleImage3", Image, queue_size=10)
        #self.pub_red_img4 = rospy.Publisher("/processedImage/detectedObjects/redCircleImage4", Image, queue_size=10)                
        #Iniciamos el voctor de posición en cero
        self.image = None
        self.rate = rospy.Rate(10)

        self.cv_image = np.zeros((300,300, 3))
        self.red1 = np.zeros((300,300))
        self.red2 = np.zeros((300,300))
        self.processedImage = np.zeros((300,280,3))
        self.processedImage2 = np.zeros((300,280))
        self.outImage = np.zeros((300,280,300))

        self.cv_image = np.uint8(self.cv_image)
        self.red1 = np.uint8(self.red1)
        self.red2 = np.uint8(self.red2)
        self.processedImage = np.uint8(self.processedImage)
        self.processedImage2 = np.uint8(self.processedImage2)
        self.outImage = np.uint8(self.outImage)

        self.processed_image_msg = Image()
        self.processed_image_msg1 = Image()
        self.processed_image_msg2 = Image()
        self.processed_image_msg3 = Image()
        self.processed_image_msg4 = Image()

        self.kernel = np.ones((8,8),np.uint8)
        self.smaller_kernel = np.ones((4,4),np.uint8)
        self.iterator = 0 

    
    def on_image_callback(self, image):
    	self.image = image

    def main(self):
    	while not rospy.is_shutdown():
    		if (self.image != None):
    			self.rate.sleep()
    			self.cv_image = self.bridge.imgmsg_to_cv2(self.image, desired_encoding="bgr8")
    			#self.cv_image = np.uint8(self.cv_image)
    			#print("image converted succesfully")
    			

                self.processedImage = cv.cvtColor(self.cv_image, cv.COLOR_BGR2HSV)               
    
                self.red1 = cv.inRange(self.processedImage,(170,70,50),(180,255,255))                
                self.red2 = cv.inRange(self.processedImage,(0,70,50),(10,255,255))                
                
                self.processedImage2 = np.zeros(self.red1.shape, dtype=np.uint8)
                
                #self.proccesedImage2 = cv.bitwise_or(self.red1,self.red2)
                condition = np.logical_not( np.logical_or(self.red1==255, self.red2==255) )
                self.processedImage2 = np.where(condition, self.processedImage2, 255)        

                self.processedImage2 = cv.morphologyEx(self.processedImage2, cv.MORPH_CLOSE, self.smaller_kernel)
                self.processedImage2 = cv.dilate(self.processedImage2,self.smaller_kernel,iterations = 1)    
                self.processedImage2 = cv.bitwise_not(self.processedImage2)
                self.processedImage2 = cv.blur(self.processedImage2, (8,8))
                self.processedImage2 = cv.blur(self.processedImage2, (8,8))


                (cols, rows) = self.processedImage2.shape
                #print("cols value is: " + str(cols))
                #print("rows value is: " + str(rows))

                blobDetectorParams = cv.SimpleBlobDetector_Params()
                blobDetectorParams.filterByCircularity = True    
                blobDetectorParams.minCircularity = 0.71
                blobDetectorParams.maxCircularity = 1.0
                blobDetectorParams.filterByArea = True
                blobDetectorParams.minArea = float((rows*cols)*0.02)
                blobDetectorParams.maxArea = float((rows*cols)*0.75)
                blobDetectorParams.filterByColor = False
                blobDetectorParams.filterByConvexity = False
                blobDetectorParams.filterByInertia = False
                
                ver = (cv.__version__).split('.')
                if int(ver[0]) < 3 :
                    detector = cv.SimpleBlobDetector(blobDetectorParams)
                else : 
                    detector = cv.SimpleBlobDetector_create(blobDetectorParams)
                
                keypoints = detector.detect(self.processedImage2)

                
                self.outImage = cv.drawKeypoints(self.processedImage2, keypoints, 0, (0,0,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

                result = Bool()                
                if(len(keypoints) != 0):                    
                    result.data = True
                    self.pub_red.publish(result)
                else:
                    result.data = False
                    self.pub_red.publish(result)

                self.processed_image_msg = self.bridge.cv2_to_imgmsg(self.outImage, encoding = "bgr8")
                self.pub_red_img.publish(self.processed_image_msg)

                self.iterator += 1


if __name__ == "__main__":
	ip = redCirclesDetector()
	ip.main()




