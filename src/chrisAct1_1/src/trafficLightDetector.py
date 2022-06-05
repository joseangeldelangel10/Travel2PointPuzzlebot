#!/usr/bin/env python
# coding=utf-8
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float64, String
import cv2 as cv
import cv_bridge



class trafficLightDetector():

    def __init__(self):
        #Inicializamos el nodo
        rospy.init_node("trafficLightDetector")       
        #susbcribers
        self.imageSubscriber = rospy.Subscriber("/video_source/raw",Image,self.on_image_callback)        
        self.bridge = cv_bridge.CvBridge()        
        #publishers
        self.pub_red_img = rospy.Publisher("/processedImage/detectedObjects/redCircleImage", Image, queue_size=10)
        self.pub_yellow_img = rospy.Publisher("/processedImage/detectedObjects/yellowCircleImage", Image, queue_size=10)
        self.pub_green_img = rospy.Publisher("/processedImage/detectedObjects/greenCircleImage", Image, queue_size=10)
        self.pub = rospy.Publisher("/curr_traffic_light", String,queue_size = 1)             
        
        self.rate = rospy.Rate(10)

        self.image = None

        self.cv_image = np.zeros((300,300, 3))
        self.cv_image = np.uint8(self.cv_image)

        #---------------------------------------------- Red Image Attributes ---------------------------------------
        
        self.red1 = np.zeros((300,300))
        self.red2 = np.zeros((300,300))
        self.processedImage_red = np.zeros((300,280,3))
        self.processedImage2_red = np.zeros((300,280))
        self.outImage_red = np.zeros((300,280,300))

        self.red1 = np.uint8(self.red1)
        self.red2 = np.uint8(self.red2)
        self.processedImage_red = np.uint8(self.processedImage_red)
        self.processedImage2_red = np.uint8(self.processedImage2_red)
        self.outImage_red = np.uint8(self.outImage_red)

        #---------------------------------------------- Yellow Image Attributes ---------------------------------------


        self.yellow = np.zeros((300,300))
        self.processedImage_yellow = np.zeros((300,280,3))
        self.processedImage2_yellow = np.zeros((300,280))
        self.outImage_yellow = np.zeros((300,280,300))

        self.yellow = np.uint8(self.yellow)
        self.processedImage_yellow = np.uint8(self.processedImage_yellow)
        self.processedImage2_yellow = np.uint8(self.processedImage2_yellow)
        self.outImage_yellow = np.uint8(self.outImage_yellow)

        #---------------------------------------------- Green Image Attributes ---------------------------------------

        self.green = np.zeros((300,300))
        self.processedImage_green = np.zeros((300,280,3))
        self.processedImage2_green = np.zeros((300,280))
        self.outImage_green = np.zeros((300,280,300))

        self.green = np.uint8(self.green)
        self.processedImage_green = np.uint8(self.processedImage_green)
        self.processedImage2_green = np.uint8(self.processedImage2_green)
        self.outImage_green = np.uint8(self.outImage_green)

        #------------------------------------------------------------------------------------------------------------

        self.processed_image_msg_red = Image()
        self.processed_image_msg_yellow = Image()
        self.processed_image_msg_green = Image()

        self.kernel = np.ones((8,8),np.uint8)
        self.smaller_kernel = np.ones((4,4),np.uint8)
         

    
    def on_image_callback(self, image):
        self.image = image

    def red_light_detected(self):

        self.rate.sleep()
        self.cv_image = self.bridge.imgmsg_to_cv2(self.image, desired_encoding="bgr8")
        self.processedImage_red = cv.cvtColor(self.cv_image, cv.COLOR_BGR2HSV)

        self.red1 = cv.inRange(self.processedImage_red,(170,70,50),(180,255,255))                
        self.red2 = cv.inRange(self.processedImage_red,(0,70,50),(10,255,255))

        self.processedImage2_red = np.zeros(self.red1.shape, dtype=np.uint8)
                
        condition = np.logical_not( np.logical_or(self.red1==255, self.red2==255) )
        self.processedImage2_red = np.where(condition, self.processedImage2_red, 255)        

        self.processedImage2_red = cv.morphologyEx(self.processedImage2_red, cv.MORPH_CLOSE, self.smaller_kernel)
        self.processedImage2_red = cv.dilate(self.processedImage2_red,self.smaller_kernel,iterations = 1)    
        self.processedImage2_red = cv.bitwise_not(self.processedImage2_red)
        self.processedImage2_red = cv.blur(self.processedImage2_red, (8,8))
        self.processedImage2_red = cv.blur(self.processedImage2_red, (8,8))

        (cols, rows) = self.processedImage2_red.shape

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
                
        keypoints = detector.detect(self.processedImage2_red)

                
        self.outImage_red = cv.drawKeypoints(self.processedImage2_red, keypoints, 0, (0,0,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        self.processed_image_msg_red = self.bridge.cv2_to_imgmsg(self.outImage_red, encoding = "bgr8")
        self.pub_red_img.publish(self.processed_image_msg_red)
               
        if(len(keypoints) != 0):                    
            return True
        else:
            return False

    def yellow_light_detected(self):

        self.rate.sleep()
        self.cv_image = self.bridge.imgmsg_to_cv2(self.image, desired_encoding="bgr8")

        self.processedImage_yellow = cv.cvtColor(self.cv_image, cv.COLOR_BGR2HSV)

        self.processedImage2_yellow = cv.inRange(self.processedImage_yellow,(20,70,50),(37,255,255))   

        self.processedImage2_yellow = cv.morphologyEx(self.processedImage2_yellow, cv.MORPH_CLOSE, self.smaller_kernel)
        self.processedImage2_yellow = cv.dilate(self.processedImage2_yellow,self.smaller_kernel,iterations = 1)    
        self.processedImage2_yellow = cv.bitwise_not(self.processedImage2_yellow)
        self.processedImage2_yellow = cv.blur(self.processedImage2_yellow, (8,8))
        self.processedImage2_yellow = cv.blur(self.processedImage2_yellow, (8,8))

        (cols, rows) = self.processedImage2_yellow.shape

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
                
        keypoints = detector.detect(self.processedImage2_yellow)

                
        self.outImage_yellow = cv.drawKeypoints(self.processedImage2_yellow, keypoints, 0, (0,0,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        self.processed_image_msg_yellow = self.bridge.cv2_to_imgmsg(self.outImage_yellow, encoding = "bgr8")
        self.pub_yellow_img.publish(self.processed_image_msg_yellow)
               
        if(len(keypoints) != 0):                    
            return True
        else:
            return False

    def green_light_detected(self):

        self.rate.sleep()
        self.cv_image = self.bridge.imgmsg_to_cv2(self.image, desired_encoding="bgr8")

        self.processedImage_green = cv.cvtColor(self.cv_image, cv.COLOR_BGR2HSV)

        self.processedImage2_green = cv.inRange(self.processedImage_green,(20,70,50),(37,255,255))   

        self.processedImage2_green = cv.morphologyEx(self.processedImage2_green, cv.MORPH_CLOSE, self.smaller_kernel)
        self.processedImage2_green = cv.dilate(self.processedImage2_green,self.smaller_kernel,iterations = 1)    
        self.processedImage2_green = cv.bitwise_not(self.processedImage2_green)
        self.processedImage2_green = cv.blur(self.processedImage2_green, (8,8))
        self.processedImage2_green = cv.blur(self.processedImage2_green, (8,8))

        (cols, rows) = self.processedImage2_green.shape

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
                
        keypoints = detector.detect(self.processedImage2_green)

                
        self.outImage_green = cv.drawKeypoints(self.processedImage2_green, keypoints, 0, (0,0,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        self.processed_image_msg_green = self.bridge.cv2_to_imgmsg(self.outImage_green, encoding = "bgr8")
        self.pub_green_img.publish(self.processed_image_msg_green)
               
        if(len(keypoints) != 0):                    
            return True
        else:
            return False

if __name__ == "__main__":

    tlDetector = trafficLightDetector()
    msg = String()

    while not rospy.is_shutdown():
        if (tlDetector.image != None):
            msgList = []
            if tlDetector.red_light_detected() or tlDetector.yellow_light_detected() or tlDetector.green_light_detected():
                if tlDetector.red_light_detected():    
                    msgList.append('red')
                if tlDetector.yellow_light_detected():    
                    msgList.append('yellow')
                if tlDetector.green_light_detected():    
                    msgList.append('green')
            else:
                msgList.append('none')

            msg.data = str(msgList)
            tlDetector.pub.publish(msg)