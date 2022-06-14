#!/usr/bin/env python
# coding=utf-8
import sys
import numpy as np
import rospy
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Pose2D
import cv2 as cv
import cv_bridge

#Creamos la clase
class crosswalkDetector():
    def __init__(self, i_width = 640, i_height = 360, camera_topic = "/video_source/raw", simulation = False):
        #Inicializamos el nodo
        rospy.init_node("crosswalkDtector")
        #Creamos el publisher
        self.pub = rospy.Publisher("/crosswalk_in_scene", Bool, queue_size=1)
        #self.pub = rospy.Publisher("/cmd_vel_LF2", Twist, queue_size=1)
        self.pub_processed_img = rospy.Publisher("/processedImage/crosswalk", Image, queue_size=10)
        
        #Creamos los subscribers
        self.imageSubscriber = rospy.Subscriber(camera_topic,Image,self.on_image_callback)

        self.bridge = cv_bridge.CvBridge()   

        self.simulation = simulation
        self.image = None   

        self.cv_image = np.zeros((300,300, 3))        
        self.gray_image = np.zeros((300,280))
        self.binary_image = np.zeros((300,280))
        self.outImage = np.zeros((300,280, 3))

        self.cv_image = np.uint8(self.cv_image)
        self.gray_image = np.uint8(self.gray_image)
        self.binary_image = np.uint8(self.binary_image)
        self.outImage = np.uint8(self.outImage)

        self.image_height = i_width
        self.image_width = i_height

        self.mask1 = None
        self.mask2 = None
        self.kernel = np.ones((20,20),np.uint8)
        
        #Declaramos que vamos a mandar 20 mensajes por segundo.
        self.rate = rospy.Rate(15)
        self.rateInt = 15

        self.crosswalk_detected_msg = Bool()
        self.processed_image_msg = Image()        
        

    def on_image_callback(self, image):
        self.image = image        

    def detectCrosswalk(self):
        self.cv_image = self.bridge.imgmsg_to_cv2(self.image, desired_encoding="bgr8")
        self.cv_image = cv.rotate(self.cv_image,cv.ROTATE_180)             
        self.gray_image = cv.cvtColor(self.cv_image, cv.COLOR_BGR2GRAY)        

        image_height = self.gray_image.shape[0]
        image_width = self.gray_image.shape[1]

        
        self.gray_image = cv.blur(self.gray_image,(3,3))
        self.gray_image = cv.blur(self.gray_image,(3,3))
        '''
        self.gray_image = cv.blur(self.gray_image,(10,10))
        '''

        thres, self.binary_image = cv.threshold(self.gray_image, 118, 255, cv.THRESH_BINARY)

        self.mask2 = np.ones(self.binary_image.shape)*255
        self.binary_image[0: image_height - int(image_height/2), :] = self.mask2[0: image_height - int(image_height/2), :]

        #mask3 = np.ones(binary_image.shape)*255
        self.binary_image[image_height-8:, :] = self.mask2[image_height-8:, :]

        self.binary_image[:, 0:5] = self.mask2[:, 0:5]
        self.binary_image[:, image_width-5:] = self.mask2[:, image_width-5:]
    
        self.binary_image = cv.morphologyEx(self.binary_image, cv.MORPH_CLOSE, self.kernel)
        self.binary_image = cv.morphologyEx(self.binary_image, cv.MORPH_CLOSE, self.kernel)
        self.binary_image = cv.bitwise_not(self.binary_image)
        

        blobs = []
        y_distances = {}
        y_cordinates_x_minmax = {}
        y_distance_tresh = 8
        if self.simulation:
            _, contours, _ = cv.findContours(self.binary_image, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        else: 
            contours, _ = cv.findContours(self.binary_image, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
        self.outImage = cv.merge((self.binary_image, self.binary_image, self.binary_image))

        #print(contours)

        for c in contours:
            x,y,w,h = cv.boundingRect(c)
            #cv.rectangle(self.outImage, (x,y), (x+w, y+h), (255, 0, 0), 2)
            center_x = x + w/2 
            center_y = y + h/2 
            blobs.append((center_x,center_y,w,h))

        if len(blobs) > 3:            
            for i in range(len(blobs)):
                for j in range(i+1,len(blobs)):
                    #perform a combination traversal trough blobs                      
                    dist_y = abs(blobs[i][1] - blobs[j][1])
                    temp_list = [blobs[i][0], blobs[j][0]]
                    dist_y_key = None
                    for dist in y_distances.keys():
                        #check if current blobs distance is similar to past distances, if not add new distance
                        if abs(dist_y-dist) < y_distance_tresh:                            
                            y_distances[dist] += 1                            
                            new_x_minmax = y_cordinates_x_minmax[dist]
                            dist_y_key = dist
                            if max(temp_list) > new_x_minmax[1]:
                                new_x_minmax[1] = max(temp_list)
                            if min(temp_list) < new_x_minmax[0]:
                                new_x_minmax[0] = min(temp_list)
                            new_x_minmax[2] = blobs[j][1]                              
                            y_cordinates_x_minmax[dist] = new_x_minmax
                        else:                            
                            y_distances[dist_y] = 1
                            y_cordinates_x_minmax[dist_y] = [min(temp_list), max(temp_list), blobs[i][1]]
                    if len(y_distances) == 0:
                        y_distances[dist_y] = 1
                        y_cordinates_x_minmax[dist_y] = [min(temp_list), max(temp_list), blobs[i][1]]
                    if 6 in y_distances.values():
                        # we use six since 4 combination 2 equals 6
                        rect_1st_corner = (y_cordinates_x_minmax[dist_y_key][0]-20, y_cordinates_x_minmax[dist_y_key][2]-20)
                        rect_2nd_corner = (y_cordinates_x_minmax[dist_y_key][1]+20, y_cordinates_x_minmax[dist_y_key][2]+20)
                        cv.rectangle(self.outImage, rect_1st_corner, rect_2nd_corner, (255, 0, 0), 2)
                        cv.putText(self.outImage,'crosswalk',(rect_1st_corner[0],rect_1st_corner[1]-10),2,0.7,(255,0,0),2,cv.LINE_AA)
                        self.processed_image_msg = self.bridge.cv2_to_imgmsg(self.outImage, encoding = "bgr8")
                        self.pub_processed_img.publish(self.processed_image_msg)
                        return True
            self.processed_image_msg = self.bridge.cv2_to_imgmsg(self.outImage, encoding = "bgr8")
            self.pub_processed_img.publish(self.processed_image_msg)
            return False       
        else:
            #NO WAY THAT IS A CROSS WALK
            self.processed_image_msg = self.bridge.cv2_to_imgmsg(self.outImage, encoding = "bgr8")
            self.pub_processed_img.publish(self.processed_image_msg)
            return False 
        

#Si el archivo es corrido directametne y no llamado desde otro archivo corremos

if __name__ == "__main__":
    #  ==================== we extract the ROS arguments passed to init the node =======================
    arguments  = rospy.myargv(argv=sys.argv)
    if len(arguments) > 1:
        if arguments[1] == "simulation":
            simulation = True
            camera_topic = "/camera/image_rotated"
            image_height = 800
            image_width = 800
        else:
            simulation = False
            camera_topic = "/video_source/raw"
            image_height = 360
            image_width = 640
    else:
        simulation = False
        camera_topic = "/video_source/raw"
        image_height = 360
        image_width = 640
    print("initial arguments are:")
    print("image size = " + str( (image_height, image_width) ))
    print("camera topic = " + camera_topic)
        
    #  ==================== ================================================== =======================
    
    #iniciamos la clase
    detector = crosswalkDetector(image_width, image_height, camera_topic, simulation)        
    while not rospy.is_shutdown():
        try:
            if detector.image != None:
                detector.crosswalk_detected_msg.data = detector.detectCrosswalk()
                detector.pub.publish(detector.crosswalk_detected_msg)    
            detector.rate.sleep()
        except rospy.ROSTimeMovedBackwardsException:
                rospy.logerr("ROS Time Backwards! - crossWalkDetector")
        