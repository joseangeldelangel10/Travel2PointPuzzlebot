#!/usr/bin/env python
# coding=utf-8
import sys
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float64, String
import cv2 as cv
import cv_bridge



class trafficLightDetector():

    def __init__(self, camera_topic = "/video_source/raw", simulation = False):
        #Inicializamos el nodo
        rospy.init_node("trafficLightDetector")       
        #susbcribers
        self.imageSubscriber = rospy.Subscriber(camera_topic,Image,self.on_image_callback)        
        self.bridge = cv_bridge.CvBridge()        
        #publishers
        self.pub_red_img = rospy.Publisher("/processedImage/detectedObjects/redCircleImage", Image, queue_size=10)
        self.pub_yellow_img = rospy.Publisher("/processedImage/detectedObjects/yellowCircleImage", Image, queue_size=10)
        self.pub_green_img = rospy.Publisher("/processedImage/detectedObjects/greenCircleImage", Image, queue_size=10)
        self.pub = rospy.Publisher("/curr_traffic_lights", String,queue_size = 1)             
        
        self.rate = rospy.Rate(10)

        self.simulation = simulation
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

    def passes_blob_filters(self, image, bound_rect_x, bound_rect_y, bound_rect_w, bound_rect_h, contour, simulation=False):
        if simulation:            
            bounding_rect_area = bound_rect_h*bound_rect_w
            minImageArea = float(image.shape[0])*float(image.shape[1])*(1.0/800.0)
            #print("image shape is: " + str(image.shape))
            #print("bounding_rect_area is {boundingRectArea} and minImageArea is {minImageArea}".format(boundingRectArea=bounding_rect_area, minImageArea=minImageArea))
            if bounding_rect_area >= minImageArea:
                bounding_box = cv.minAreaRect(contour)                                
                if abs(bounding_box[1][0]-bounding_box[1][1]) <= abs(bounding_box[1][0]*(0.25)): 
                    return True
            return False
        else:
            return None

    def array2string(self, array):
        if len(array) >= 1: 
            result = "\'  ,  \'".join(array)
            result = "[\'" + result
            result += "\']"
            return result
        else:
            return "[]"

    def red_light_detected(self):

        #self.rate.sleep() # ==========================> !
        self.cv_image = self.bridge.imgmsg_to_cv2(self.image, desired_encoding="bgr8")
        self.cv_image = cv.rotate(self.cv_image,cv.ROTATE_180)
        self.processedImage_red = cv.cvtColor(self.cv_image, cv.COLOR_BGR2HSV)

        self.red1 = cv.inRange(self.processedImage_red,(170,70,50),(180,255,255))                
        self.red2 = cv.inRange(self.processedImage_red,(0,70,50),(10,255,255))

        self.processedImage2_red = np.zeros(self.red1.shape, dtype=np.uint8)
                
        condition = np.logical_not( np.logical_or(self.red1==255, self.red2==255) )
        self.processedImage2_red = np.where(condition, self.processedImage2_red, 255)        

        self.processedImage2_red = cv.morphologyEx(self.processedImage2_red, cv.MORPH_CLOSE, self.smaller_kernel)
        self.processedImage2_red = cv.dilate(self.processedImage2_red,self.smaller_kernel,iterations = 1)    
        
        if not self.simulation:
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
        else:
            keypoints = []
            self.outImage_red = cv.merge((self.processedImage2_red, self.processedImage2_red, self.processedImage2_red))
            _, contours, _ = cv.findContours(self.processedImage2_red, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            for c in contours:
                x,y,w,h = cv.boundingRect(c)
                if self.passes_blob_filters(self.processedImage2_red, x, y, w, h, c, simulation = True):
                    cv.rectangle(self.outImage_red, (x,y), (x+w, y+h), (255, 0, 0), 2)
                    cv.putText(self.outImage_red,'red_light',(x,y-10),2,0.7,(255,0,0),2,cv.LINE_AA)
                    keypoints.append((x,y,w,h))


        self.processed_image_msg_red = self.bridge.cv2_to_imgmsg(self.outImage_red, encoding = "bgr8")
        self.pub_red_img.publish(self.processed_image_msg_red)
               
        if(len(keypoints) != 0):                    
            return True
        else:
            return False

    def yellow_light_detected(self):

        #self.rate.sleep()
        self.cv_image = self.bridge.imgmsg_to_cv2(self.image, desired_encoding="bgr8")
        self.cv_image = cv.rotate(self.cv_image,cv.ROTATE_180)
        self.processedImage_yellow = cv.cvtColor(self.cv_image, cv.COLOR_BGR2HSV)

        self.processedImage2_yellow = cv.inRange(self.processedImage_yellow,(20,70,50),(37,255,255))   

        self.processedImage2_yellow = cv.morphologyEx(self.processedImage2_yellow, cv.MORPH_CLOSE, self.smaller_kernel)
        self.processedImage2_yellow = cv.dilate(self.processedImage2_yellow,self.smaller_kernel,iterations = 1)    
        if not self.simulation:
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
        else:
            keypoints = []
            self.outImage_yellow = cv.merge((self.processedImage2_yellow, self.processedImage2_yellow, self.processedImage2_yellow))
            _, contours, _ = cv.findContours(self.processedImage2_yellow, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            for c in contours:
                x,y,w,h = cv.boundingRect(c)
                if self.passes_blob_filters(self.processedImage2_yellow, x, y, w, h, c, simulation = True):
                    cv.rectangle(self.outImage_yellow, (x,y), (x+w, y+h), (255, 0, 0), 2)
                    cv.putText(self.outImage_yellow,'yellow_light',(x,y-10),2,0.7,(255,0,0),2,cv.LINE_AA)
                    keypoints.append((x,y,w,h))

        self.processed_image_msg_yellow = self.bridge.cv2_to_imgmsg(self.outImage_yellow, encoding = "bgr8")
        self.pub_yellow_img.publish(self.processed_image_msg_yellow)
               
        if(len(keypoints) != 0):                    
            return True
        else:
            return False

    def green_light_detected(self):

        #self.rate.sleep()
        self.cv_image = self.bridge.imgmsg_to_cv2(self.image, desired_encoding="bgr8")
        self.cv_image = cv.rotate(self.cv_image,cv.ROTATE_180)
        self.processedImage_green = cv.cvtColor(self.cv_image, cv.COLOR_BGR2HSV)

        self.processedImage2_green = cv.inRange(self.processedImage_green,(38,50,40),(85,255,255))   

        self.processedImage2_green = cv.morphologyEx(self.processedImage2_green, cv.MORPH_CLOSE, self.smaller_kernel)
        self.processedImage2_green = cv.dilate(self.processedImage2_green,self.smaller_kernel,iterations = 1)    
        
        if not self.simulation:
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
        else:
            keypoints = []
            self.outImage_green = cv.merge((self.processedImage2_green, self.processedImage2_green, self.processedImage2_green))
            _, contours, _ = cv.findContours(self.processedImage2_green, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            for c in contours:
                x,y,w,h = cv.boundingRect(c)
                if self.passes_blob_filters(self.processedImage2_green, x, y, w, h, c, simulation = True):
                    cv.rectangle(self.outImage_green, (x,y), (x+w, y+h), (255, 0, 0), 2)
                    cv.putText(self.outImage_green,'green_light',(x,y-10),2,0.7,(255,0,0),2,cv.LINE_AA)
                    keypoints.append((x,y,w,h))

        self.processed_image_msg_green = self.bridge.cv2_to_imgmsg(self.outImage_green, encoding = "bgr8")
        self.pub_green_img.publish(self.processed_image_msg_green)
               
        if(len(keypoints) != 0):                    
            return True
        else:
            return False

if __name__ == "__main__":
    #  ==================== we extract the ROS arguments passed to init the node =======================
    arguments  = rospy.myargv(argv=sys.argv)
    if len(arguments) > 1:
        if arguments[1] == "simulation":
            sim = True
            c_topic = "/camera/image_rotated"            
        else:
            sim = False
            c_topic = "/video_source/raw"
    else:
        sim = False
        c_topic = "/video_source/raw"
    print("initial arguments are:")    
    print("camera topic = " + c_topic)
        
    #  ==================== ================================================== =======================

    tlDetector = trafficLightDetector(camera_topic = c_topic, simulation = sim)
    msg = String()

    while not rospy.is_shutdown():
        if (tlDetector.image != None):
            msgList = []            
            if tlDetector.red_light_detected():    
                msgList.append('red traffic light')
            if tlDetector.yellow_light_detected():    
                msgList.append('yellow traffic light')
            if tlDetector.green_light_detected():    
                msgList.append('green traffic light')            

            msg.data = tlDetector.array2string(msgList)
            tlDetector.pub.publish(msg)
        tlDetector.rate.sleep()