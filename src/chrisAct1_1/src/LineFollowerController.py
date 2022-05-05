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
        rospy.init_node("LineFollowerController")
        #Creamos el publisher
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pub_processed_img = rospy.Publisher("/processedImage/ROI", Image, queue_size=10)
        
        #Creamos los subscribers
        self.imageSubscriber = rospy.Subscriber("/video_source/raw",Image,self.on_image_callback)

        self.bridge = cv_bridge.CvBridge()      

        self.cv_image = np.zeros((300,300, 3))        
        self.gray_image = np.zeros((300,280))
        self.binary_image = np.zeros((300,280))
        self.outImage = np.zeros((300,280, 3))

        self.cv_image = np.uint8(self.cv_image)
        self.gray_image = np.uint8(self.gray_image)
        self.binary_image = np.uint8(self.binary_image)
        self.outImage = np.uint8(self.outImage)

        self.mask1 = None
        self.mask2 = None
        self.kernel = np.ones((10,10),np.uint8)
        
        #Declaramos que vamos a mandar 20 mensajes por segundo.
        self.rate = rospy.Rate(1)
        self.msg = Twist()
        self.processed_image_msg = Image()

        self.velocities_queue = []
        for i in range(6):
            # seven since 43cm/7cm = 6.14
            self.velocities_queue.append( (0.07, 0.0) ) #(vlin, w)        

        #self.state = "travelingTowardsGoal"
        
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
        self.gray_image = cv.cvtColor(self.cv_image, cv.COLOR_BGR2GRAY)

        image_height = self.gray_image.shape[0]
        image_width = self.gray_image.shape[1]

        self.mask1 = np.ones(self.gray_image.shape)*127 # this mask removes upper half image noise
        self.gray_image[ 0: int(image_height/2), :] = self.mask1[ 0: int(image_height/2), :]

        thres, self.binary_image = cv.threshold(self.gray_image, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)

        self.mask2 = np.ones(self.binary_image.shape)*255
        self.binary_image[0: image_height - 40, :] = self.mask2[0: image_height - 40, :]

        #mask3 = np.ones(binary_image.shape)*255
        self.binary_image[image_height-2:, :] = self.mask2[image_height-2:, :]
    
        self.binary_image = cv.morphologyEx(self.binary_image, cv.MORPH_CLOSE, self.kernel)
        #binary_image = cv.erode(binary_image, kernel, iterations = 2)    

        blobDetectorParams = cv.SimpleBlobDetector_Params()
                
        ver = (cv.__version__).split('.')
        if int(ver[0]) < 3 :
            detector = cv.SimpleBlobDetector(blobDetectorParams)
        else : 
            detector = cv.SimpleBlobDetector_create(blobDetectorParams)
                    
        keypoints = detector.detect(self.binary_image)

        self.outImage = cv.drawKeypoints(self.binary_image, keypoints, 0, (0, 0,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        self.processed_image_msg = self.bridge.cv2_to_imgmsg(self.outImage, encoding = "bgr8")
        self.pub_processed_img.publish(self.processed_image_msg)

        if len(keypoints == 1):
            return [ keypoints[0].pt[0], keypoints[0].pt[1] ]
        else:
            return "Error" 

 


#Si el archivo es corrido directametne y no llamado desde otro archivo corremos


if __name__ == "__main__":
    #iniciamos la clase
    follower = LineFollowerController()
    #mientras este corriendo el nodo movemos el carro el circulo
    Kpw = 0.5

    while not rospy.is_shutdown():
        #Llamamos el sleep para asegurar los 20 msg por segundo
        
        if follower.image != None:
            blob_cord = follower.detectROI()
            if blob_cord != "Error":
                e1 = blob_cord[0]
                e2 = follower.image_width - e1
                new_w = kpw*(e2-e1)
                new_v = 0.07
            else:
                new_w = 0.0
                new_v = 0.0            
            follower.velocities_queue.append( (new_v, new_w) )
            follower.move( follower.velocities_queue[0][0], follower.velocities_queue[0][1] )
            follower.velocities_queue = follower.velocities_queue[1:]
            
    

                


            
        follow.rate.sleep()

            
    #mov.main()
