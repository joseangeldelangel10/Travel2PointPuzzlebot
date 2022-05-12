#!/usr/bin/env python
# coding=utf-8
import sys
import numpy as np
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Pose2D
import cv2 as cv
import cv_bridge

#Creamos la clase
class LineFollowerController():
    def __init__(self):
        #Inicializamos el nodo
        rospy.init_node("LaneFollowerController")
        #Creamos el publisher
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pub_processed_img = rospy.Publisher("/processedImage/ROI", Image, queue_size=10)
        
        #Creamos los subscribers
        self.imageSubscriber = rospy.Subscriber("/video_source/raw",Image,self.on_image_callback)        

        self.bridge = cv_bridge.CvBridge()   

        self.image = None   

        self.cv_image = np.zeros((300,300, 3))        
        self.gray_image = np.zeros((300,280))
        self.binary_image = np.zeros((300,280))
        self.outImage = np.zeros((300,280, 3))

        self.cv_image = np.uint8(self.cv_image)
        self.gray_image = np.uint8(self.gray_image)
        self.binary_image = np.uint8(self.binary_image)
        self.outImage = np.uint8(self.outImage)

        self.image_height = 480
        self.image_width = 720
        self.image_width_in_cm = 9

        self.mask1 = None
        self.mask2 = None
        self.kernel = np.ones((20,20),np.uint8)
        
        #Declaramos que vamos a mandar 20 mensajes por segundo.
        self.rate = rospy.Rate(20)
        self.rateInt = 20
        self.msg = Twist()
        self.processed_image_msg = Image()

        self.last_point = [self.image_width/2, self.image_height/2]       

        self.state = "common"        
        self.time2turn = 4 #4 sec
        self.cicles2Turn = self.time2turn*self.rateInt
        self.turnCounter = 1
        self.roiSize = 80
        #Creamos un función de que hacer cuando haya un shutdown        
        rospy.on_shutdown(self.end_callback)

    def red_circle_callback(self, data):
        self.redCircleDetected = data.data

    def green_circle_callback(self, data):
        self.greenCircleDetected = data.data

    def yellow_circle_callback(self, data):
        self.yellowCircleDetected = data.data

    def on_image_callback(self, image):
        self.image = image

    def move(self,v_lin,v_ang):
        """Funcion que publica la velocidad lineal y angular en el puzzlebot"""
        #Declaramos le velocidad deseada en el mensaje tipo Twist
        self.msg.linear.x = v_lin
        self.msg.linear.y = 0.0
        self.msg.linear.z = 0.0
        self.msg.angular.x = 0.0
        self.msg.angular.y = 0.0
        self.msg.angular.z = v_ang
        #Publicamos la velocidad
        self.pub.publish(self.msg)


    def end_callback(self):
        """Funcion que para el puzzlebot cuando el nodo deja de ser corrido"""
        #Declaramos las velocidades de cero
        self.msg.linear.x = 0.0
        self.msg.angular.z = 0.0
        #Publicamos las velocidades
        self.pub.publish(self.msg)

    def detectROI(self):

        self.cv_image = self.bridge.imgmsg_to_cv2(self.image, desired_encoding="bgr8")
        self.cv_image = cv.rotate(self.cv_image,cv.ROTATE_180)             
        self.gray_image = cv.cvtColor(self.cv_image, cv.COLOR_BGR2GRAY)

        image_height = self.gray_image.shape[0]
        image_width = self.gray_image.shape[1]

        self.mask1 = np.ones(self.gray_image.shape)*127 # this mask removes upper half image noise
        self.gray_image[ 0: int(image_height/2), :] = self.mask1[ 0: int(image_height/2), :]

        thres, self.binary_image = cv.threshold(self.gray_image, 80, 255, cv.THRESH_BINARY)

        self.mask2 = np.ones(self.binary_image.shape)*255
        self.binary_image[0: image_height - self.roiSize, :] = self.mask2[0: image_height - self.roiSize, :]

        #mask3 = np.ones(binary_image.shape)*255
        self.binary_image[image_height-8:, :] = self.mask2[image_height-8:, :]

        self.binary_image[:, 0:5] = self.mask2[:, 0:5]
        self.binary_image[:, image_width-5:] = self.mask2[:, image_width-5:]
    
        self.binary_image = cv.morphologyEx(self.binary_image, cv.MORPH_CLOSE, self.kernel)
        self.binary_image = cv.morphologyEx(self.binary_image, cv.MORPH_CLOSE, self.kernel)
        self.binary_image = cv.bitwise_not(self.binary_image)        


        blobs = [] # list of tuples (center_x, center_y, w, h)
        contours, _ = cv.findContours(self.binary_image, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        self.outImage = self.binary_image

        #print(contours)

        for c in contours:
            x,y,w,h = cv.boundingRect(c)
            cv.rectangle(self.outImage, (x,y), (x+w, y+h), (255, 0, 0), 2)
            center_x = x + (x+w)/2
            center_y = y + (y+h)/2
            blobs.append((center_x,center_y,w,h))

        #we filter the blobs by area
        new_blobs = []
        for blob in blobs:
            if blob[2]*blob[3] > (image_height*image_width)*(1/9):
                new_blobs.append(blob)
        blobs = new_blobs

        blobs_center_x = []
        for b in blobs:
            blobs_center_x.append(b[0])
        #self.outImage = cv.drawKeypoints(self.binary_image, keypoints, 0, (0, 0,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        self.processed_image_msg = self.bridge.cv2_to_imgmsg(self.outImage, encoding = "mono8")
        self.pub_processed_img.publish(self.processed_image_msg)

        min_dist = image_height*image_width
        res_x = None
        res_y = None
        if len(blobs) >= 2:            
            for i in range(len(blobs)):
                for j in range(i+1,len(blobs)): 
                    cord_x = (blobs[i][0] + blobs[j][0])/2 
                    cord_y = (blobs[i][1] + blobs[j][1])/2
                    dist = np.sqrt((cord_x-self.last_point[0])**2 + (cord_y-self.last_point[1])**2)
                    if dist < min_dist:
                        min_dist = dist 
                        res_x = cord_x
                        res_y = cord_y                    
        elif len(blobs) == 1 and blobs_center_x[0] >= image_width/2:
            res_x = (-(16*(image_width/self.image_width_in_cm) + blobs_center_x[0]) + blobs_center_x[0])/2
            res_y = blobs[0][1]
        elif len(blobs) == 1 and blobs_center_x[0] < image_width/2:
            res_x = ((image_width + (16*(image_width/self.image_width_in_cm) + blobs_center_x[0])) + blobs_center_x[0])/2
            res_y = blobs[0][1]
        else:
            res_x = image_width/2
            res_y = self.roiSize/2 
        self.last_point = [res_x, res_y]
        return [ res_x, res_y ]

#Si el archivo es corrido directametne y no llamado desde otro archivo corremos
if __name__ == "__main__":
    #iniciamos la clase
    follower = LineFollowerController()
    #mientras este corriendo el nodo movemos el carro el circulo
    kpw = 0.0005
    #rospy.sleep(5)
    while not rospy.is_shutdown():
        #Llamamos el sleep para asegurar los 20 msg por segundo
        
        if follower.image != None:
            blob_cord = follower.detectROI()          

            if blob_cord != "Error":
                e1 = blob_cord[0]
                e2 = follower.image_width - e1
                if e1 < follower.image_width*(1/8):
                    follower.state = "turningLeft"                    
                elif e2 < follower.image_width*(1/8):
                    follower.state = "turningRight"
                else:
                    follower.state = "common"                    
            else:
                follower.state = "stopped"                 
            
                              
            if follower.state == "turningRight":                    
                    new_v = 0.0
                    new_w = -(np.pi/2)/follower.time2turn
                    if follower.cicles2Turn - follower.turnCounter > 0:                        
                        follower.turnCounter += 1
                    else:
                        follower.state = "common"
                        follower.turnCounter = 1
            elif follower.state == "turningLeft":
                    new_v = 0.0
                    new_w = (np.pi/2)/follower.time2turn
                    if follower.cicles2Turn - follower.turnCounter > 0:                        
                        follower.turnCounter += 1
                    else:
                        follower.state = "common"
                        follower.turnCounter = 1
            elif follower.state == "common":                                    
                    new_v = 0.15                       
                    new_w = kpw*(e2-e1)
                    if new_w >= 0.3:
                        new_w = 0.29
                    elif new_w <= -0.3:
                        new_w = -0.29                      
            elif follower.state == "stopped":
                new_w = 0.0
                new_v = 0.0
             
                        
            follower.move( new_v, new_w )
            
        follower.rate.sleep()

            
    #mov.main()
