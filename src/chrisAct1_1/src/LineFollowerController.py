#!/usr/bin/env python
# coding=utf-8
import sys
import numpy as np
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Pose2D
import cv2 as cv
import cv_bridge

#Creamos la clase
class LineFollowerController():
    def __init__(self):
        #Inicializamos el nodo
        rospy.init_node("circulo_mov_traffic_lights")
        #Creamos el publisher
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pub_red_img = rospy.Publisher("/processedImage/ROI", Image, queue_size=10)
        
        #Creamos los subscribers
        self.imageSubscriber = rospy.Subscriber("/video_source/raw",Image,self.on_image_callback)

        self.bridge = cv_bridge.CvBridge()      

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

        

        self.kernel = np.ones((8,8),np.uint8)
        self.smaller_kernel = np.ones((4,4),np.uint8)
        self.iterator = 0 
        

        #Declaramos que vamos a mandar 20 mensajes por segundo.
        self.rate = rospy.Rate(100)

        self.msg = Twist()
        self.processed_image_msg = Image()

        self.image = None
        
        self.state = "travelingTowardsGoal"
        
        #Creamos un función de que hacer cuando haya un shutdown
        rospy.on_shutdown(self.end_callback)


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

 


#Si el archivo es corrido directametne y no llamado desde otro archivo corremos


if __name__ == "__main__":
    #iniciamos la clase
    follow = LineFollowerController()
    #mientras este corriendo el nodo movemos el carro el circulo
    Kpw = 1.0
    Kpv = 0.25
    while not rospy.is_shutdown():
        #Llamamos el sleep para asegurar los 20 msg por segundo
        
        if follow.image != None:
            #print("None condition passed")
 

                


            
        follow.rate.sleep()

            
    #mov.main()
