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
        
        self.rate = rospy.Rate(30)

        self.simulation = simulation
        self.image = None

        self.cv_image = np.zeros((30,30, 3))
        self.cv_image = np.uint8(self.cv_image)

        #---------------------------------------------- Red Image Attributes ---------------------------------------
        
        self.red1 = np.zeros((30,30), dtype = np.uint8)
        self.red2 = np.zeros((30,30), dtype = np.uint8)
        self.processedImage_red = np.zeros((30,30), dtype = np.uint8)
        #self.processedImage2_red = np.zeros((30,30), dtype = np.uint8)
        self.outImage_red = np.zeros((30,30,3), dtype = np.uint8)        

        #---------------------------------------------- Yellow Image Attributes ---------------------------------------


        self.yellow = np.zeros((30,30), dtype = np.uint8)
        self.processedImage_yellow = np.zeros((30,30), dtype = np.uint8)
        #self.processedImage2_yellow = np.zeros((30,30), dtype = np.uint8)
        self.outImage_yellow = np.zeros((30,30,3), dtype = np.uint8)

        #---------------------------------------------- Green Image Attributes ---------------------------------------

        self.green = np.zeros((30,30), dtype = np.uint8)
        self.processedImage_green = np.zeros((30,30), dtype = np.uint8)
        #self.processedImage2_green = np.zeros((300,280), dtype = np.uint8)
        self.outImage_green = np.zeros((30,30,3), dtype = np.uint8)

        #------------------------------------------------------------------------------------------------------------

        self.processed_image_msg_red = Image()
        self.processed_image_msg_yellow = Image()
        self.processed_image_msg_green = Image()

        #self.kernel = np.ones((8,8),np.uint8)
        #self.smaller_kernel = np.ones((4,4),np.uint8)
        self.kernel = np.ones((4,4),np.uint8)
        self.smaller_kernel = np.ones((2,2),np.uint8)

    
    def on_image_callback(self, image):
        self.image = image

    def passes_blob_filters(self, image, bound_rect_x, bound_rect_y, bound_rect_w, bound_rect_h, contour, simulation=False):
        if simulation:            
            bounding_rect_area = bound_rect_h*bound_rect_w
            minImageArea = float(image.shape[0])*float(image.shape[1])*(1.0/900.0)
            #print("image shape is: " + str(image.shape))
            #print("bounding_rect_area is {boundingRectArea} and minImageArea is {minImageArea}".format(boundingRectArea=bounding_rect_area, minImageArea=minImageArea))
            if bounding_rect_area >= minImageArea:
                bounding_box = cv.minAreaRect(contour)                                
                if abs(bounding_box[1][0]-bounding_box[1][1]) <= abs(bounding_box[1][0]*(0.25)) and abs(bounding_box[2]) >= 0.05: 
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

    def red_light_detected(self, cv_image, minImageArea):

        self.red1 = cv.inRange(cv_image,(170,70,50),(180,255,255))                
        self.red2 = cv.inRange(cv_image,(0,70,50),(10,255,255))

        self.processedImage_red = np.zeros(self.red1.shape, dtype=np.uint8)
                
        condition = np.logical_not( np.logical_or(self.red1==255, self.red2==255) )
        self.processedImage_red = np.where(condition, self.processedImage_red, 255)        

        self.processedImage_red = cv.morphologyEx(self.processedImage_red, cv.MORPH_CLOSE, self.smaller_kernel)
        self.processedImage_red = cv.erode(self.processedImage_red,self.smaller_kernel,iterations = 2)
        self.processedImage_red = cv.dilate(self.processedImage_red,self.smaller_kernel,iterations = 5)      

        if not self.simulation:
            keypoints = []
            self.outImage_red = cv.merge((self.processedImage_red, self.processedImage_red, self.processedImage_red))
            contours, _ = cv.findContours(self.processedImage_red, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            for c in contours:
                (min_circle_x,min_circle_y),min_circle_radius = cv.minEnclosingCircle(c)
                bounding_box = cv.minAreaRect(c)
                bounding_box_x = bounding_box[1][0]
                bounding_box_y = bounding_box[1][1]
                x,y,w,h = cv.boundingRect(c)
                #if abs((min_circle_radius*2)-bounding_box_x) <= min_circle_radius*0.45 and abs((min_circle_radius*2)-bounding_box_y) <= min_circle_radius*0.45:
                #square_area = bounding_box_x*bounding_box_y
                square_area = w*h
                circle_area = np.pi*(min_circle_radius**2)
                if square_area >= minImageArea:
                    cv.rectangle(self.outImage_red, (x,y), (x+w, y+h), (0, 0, 255), 2)
                    cv.putText(self.outImage_red,'red_light',(x,y-10),2,0.7,(0,0,255),2,cv.LINE_AA)
                    keypoints.append((x,y,w,h))
        else:
            keypoints = []
            self.outImage_red = cv.merge((self.processedImage_red, self.processedImage_red, self.processedImage_red))
            _, contours, _ = cv.findContours(self.processedImage_red, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            for c in contours:
                x,y,w,h = cv.boundingRect(c)
                if self.passes_blob_filters(self.processedImage_red, x, y, w, h, c, simulation = True):
                    cv.rectangle(self.outImage_red, (x,y), (x+w, y+h), (255, 0, 0), 2)
                    cv.putText(self.outImage_red,'red_light',(x,y-10),2,0.7,(255,0,0),2,cv.LINE_AA)
                    keypoints.append((x,y,w,h))


        #self.processed_image_msg_red = self.bridge.cv2_to_imgmsg(self.outImage_red, encoding = "bgr8")
        #self.pub_red_img.publish(self.processed_image_msg_red)
               
        if(len(keypoints) != 0):                    
            return True
        else:
            return False

    def yellow_light_detected(self, cv_image, minImageArea):

        #self.rate.sleep()

        self.processedImage_yellow = cv.inRange(cv_image,(20,50,50),(37,255,255))   

        self.processedImage_yellow = cv.morphologyEx(self.processedImage_yellow, cv.MORPH_CLOSE, self.smaller_kernel)
        self.processedImage_yellow = cv.erode(self.processedImage_yellow,self.smaller_kernel,iterations = 2) 
        self.processedImage_yellow = cv.dilate(self.processedImage_yellow,self.smaller_kernel,iterations = 5) 
        
        if not self.simulation:
            keypoints = []
            self.outImage_yellow = cv.merge((self.processedImage_yellow, self.processedImage_yellow, self.processedImage_yellow))
            contours, _ = cv.findContours(self.processedImage_yellow, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            for c in contours:
                (min_circle_x,min_circle_y),min_circle_radius = cv.minEnclosingCircle(c)
                bounding_box = cv.minAreaRect(c)
                bounding_box_x = bounding_box[1][0]
                bounding_box_y = bounding_box[1][1]
                x,y,w,h = cv.boundingRect(c)
                #if abs((min_circle_radius*2)-bounding_box_x) <= min_circle_radius*0.35 and abs((min_circle_radius*2)-bounding_box_y) <= min_circle_radius*0.35:
                #square_area = bounding_box_x*bounding_box_y
                square_area = w*h
                circle_area = np.pi*(min_circle_radius**2)
                if square_area >= minImageArea:
                    cv.rectangle(self.outImage_yellow, (x,y), (x+w, y+h), (0, 255, 255), 2)
                    cv.putText(self.outImage_yellow,'yellow_light',(x,y-10),2,0.7,(0,255,255),2,cv.LINE_AA)
                    keypoints.append((x,y,w,h))
        else:
            keypoints = []
            self.outImage_yellow = cv.merge((self.processedImage_yellow, self.processedImage_yellow, self.processedImage_yellow))
            _, contours, _ = cv.findContours(self.processedImage_yellow, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            for c in contours:
                x,y,w,h = cv.boundingRect(c)
                if self.passes_blob_filters(self.processedImage_yellow, x, y, w, h, c, simulation = True):
                    cv.rectangle(self.outImage_yellow, (x,y), (x+w, y+h), (255, 0, 0), 2)
                    cv.putText(self.outImage_yellow,'yellow_light',(x,y-10),2,0.7,(255,0,0),2,cv.LINE_AA)
                    keypoints.append((x,y,w,h))

        #self.processed_image_msg_yellow = self.bridge.cv2_to_imgmsg(self.outImage_yellow, encoding = "bgr8")
        #self.pub_yellow_img.publish(self.processed_image_msg_yellow)
               
        if(len(keypoints) != 0):                    
            return True
        else:
            return False

    def green_light_detected(self, cv_image, minImageArea):

        self.processedImage_green = cv.inRange(cv_image,(38,40,40),(85,255,255))   

        self.processedImage_green = cv.morphologyEx(self.processedImage_green, cv.MORPH_CLOSE, self.smaller_kernel)
        self.processedImage_green = cv.erode(self.processedImage_green,self.smaller_kernel,iterations = 2)
        self.processedImage_green = cv.dilate(self.processedImage_green,self.smaller_kernel,iterations = 5)    
        
        if not self.simulation:
            keypoints = []
            self.outImage_green = cv.merge((self.processedImage_green, self.processedImage_green, self.processedImage_green))
            contours, _ = cv.findContours(self.processedImage_green, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            for c in contours:
                (min_circle_x,min_circle_y),min_circle_radius = cv.minEnclosingCircle(c)
                bounding_box = cv.minAreaRect(c)
                bounding_box_x = bounding_box[1][0]
                bounding_box_y = bounding_box[1][1]
                x,y,w,h = cv.boundingRect(c)
                #if abs((min_circle_radius*2)-bounding_box_x) <= min_circle_radius*0.35 and abs((min_circle_radius*2)-bounding_box_y) <= min_circle_radius*0.35:
                #square_area = bounding_box_x*bounding_box_y
                square_area = w*h
                circle_area = np.pi*(min_circle_radius**2)
                if square_area >= minImageArea:
                    cv.rectangle(self.outImage_green, (x,y), (x+w, y+h), (0, 255, 0), 2)
                    cv.putText(self.outImage_green,'green_light',(x,y-10),2,0.7,(0,255,0),2,cv.LINE_AA)
                    keypoints.append((x,y,w,h))
        else:
            keypoints = []
            self.outImage_green = cv.merge((self.processedImage_green, self.processedImage_green, self.processedImage_green))
            _, contours, _ = cv.findContours(self.processedImage_green, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            for c in contours:
                x,y,w,h = cv.boundingRect(c)
                if self.passes_blob_filters(self.processedImage_green, x, y, w, h, c, simulation = True):
                    cv.rectangle(self.outImage_green, (x,y), (x+w, y+h), (255, 0, 0), 2)
                    cv.putText(self.outImage_green,'green_light',(x,y-10),2,0.7,(255,0,0),2,cv.LINE_AA)
                    keypoints.append((x,y,w,h))

        #self.processed_image_msg_green = self.bridge.cv2_to_imgmsg(self.outImage_green, encoding = "bgr8")
        #self.pub_green_img.publish(self.processed_image_msg_green)
               
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
            tlDetector.cv_image = tlDetector.bridge.imgmsg_to_cv2(tlDetector.image, desired_encoding="bgr8")
            tlDetector.cv_image = cv.rotate(tlDetector.cv_image,cv.ROTATE_180)
            tlDetector.cv_image = cv.cvtColor(tlDetector.cv_image, cv.COLOR_BGR2HSV)
            tlDetector.cv_image = cv.resize(tlDetector.cv_image,(int(tlDetector.cv_image.shape[1]/2),int(tlDetector.cv_image.shape[0]/2)), interpolation = cv.INTER_AREA)
            minImageArea = float(tlDetector.cv_image.shape[0])*float(tlDetector.cv_image.shape[1])*float(1.0/1000.0)

            if tlDetector.red_light_detected(tlDetector.cv_image, minImageArea):    
                msgList.append('red traffic light')
            elif tlDetector.yellow_light_detected(tlDetector.cv_image, minImageArea):    
                msgList.append('yellow traffic light')
            elif tlDetector.green_light_detected(tlDetector.cv_image, minImageArea):    
                msgList.append('green traffic light')            

            msg.data = tlDetector.array2string(msgList)
            tlDetector.pub.publish(msg)
        tlDetector.rate.sleep()

