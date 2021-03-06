#!/usr/bin/env python3
# coding=utf-8
import cv2 as cv
import sys
import numpy as np
import tensorflow as tf
from tensorflow.keras.models import load_model
from tensorflow.keras import backend as bk
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Header
#import cv_bridge
#from roseus.msg import StringStamped

#Creamos la clase
class trafficSignalsDetector():
    def __init__(self, cam_topic = "/video_source/raw", simulation = False):
        #Inicializamos el nodo
        rospy.init_node("signalsDetector")        
        #Creamos el publisher
        self.pub_curr_signs = rospy.Publisher("/curr_traffic_signs", String, queue_size=1)
        self.pub_curr_signs_image = rospy.Publisher("/signs_detection_results", Image, queue_size=1)
        
        #Creamos los subscribers
        self.sub_image = rospy.Subscriber(cam_topic,Image, self.on_image_callback)
        
        self.curr_traffic_signs = []
        self.kernel = np.ones((5,5), np.uint8)
        self.cnn_probability_threshold = 0.90
        self.cv_original_image = np.zeros((300,300,3))
        self.cv_original_image = np.uint8(self.cv_original_image)
        self.cv_image = np.zeros((300,300,3))         
        self.cv_image = np.uint8(self.cv_image)
        self.haar_detection = np.zeros((300,300,3))         
        self.haar_detection = np.uint8(self.haar_detection)
        self.newWidth = None
        self.newHeight = None   
        self.simulation = simulation 
        self.image = None                
        self.curr_signs_msg = String()
        self.curr_signs_image_msg = Image()
        #self.bridge = cv_bridge.CvBridge()

        #Declaramos que vamos a mandar 20 mensajes por segundo.

        """self.red_hexagons_classifier = cv.CascadeClassifier("/home/jose/Documents/6toSemestre/chrisAct1/src/challenge_autonomous_nav/ai_models/haar/redHexagons.xml")
        self.blue_circles_classifier = cv.CascadeClassifier("/home/jose/Documents/6toSemestre/chrisAct1/src/challenge_autonomous_nav/ai_models/haar/blueCircles.xml")
        self.black_circles_classifier = cv.CascadeClassifier("/home/jose/Documents/6toSemestre/chrisAct1/src/challenge_autonomous_nav/ai_models/haar/blackCircles.xml")"""
        #path1 = "/home/jose/Documents/6toSemestre/chrisAct1/src/challenge_autonomous_nav/ai_models/haar/stopSign.xml"
        #path2 = "/home/jose/Documents/6toSemestre/chrisAct1/src/challenge_autonomous_nav/ai_models/haar/goStraight.xml"
        #path3 = "/home/jose/Documents/6toSemestre/chrisAct1/src/challenge_autonomous_nav/ai_models/haar/blueCircles.xml"
        #path4 = "/home/jose/Documents/6toSemestre/chrisAct1/src/challenge_autonomous_nav/ai_models/cnns/traffic_signs_cnn_model_5/my_model"
        path1 = "/home/puzzlebot/LeoNJosews/src/challenge_autonomous_nav/ai_models/haar/stopSign.xml"
        path2 = "/home/puzzlebot/LeoNJosews/src/challenge_autonomous_nav/ai_models/haar/goStraight.xml"
        path3 = "/home/puzzlebot/LeoNJosews/src/challenge_autonomous_nav/ai_models/haar/blueCircles.xml"
        path4 = "/home/puzzlebot/LeoNJosews/src/challenge_autonomous_nav/ai_models/cnns/traffic_signs_cnn_model_1_5/my_model"
        self.stopSignClassifier = cv.CascadeClassifier(path1)
        self.goStraightClassifier = cv.CascadeClassifier(path2)
        self.turnRightAheadClassifier = cv.CascadeClassifier(path3)
        print("casscade classifiers loaded")
        self.cnn_model = load_model(path4)
        print("cnn loaded")


        self.rate = rospy.Rate(1)
        self.rateInt = 1
        print("end of init reached")                                              

    def on_image_callback(self, image):
        self.image = image  

    def cnn_preprocessing(self, img):        
        img = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
        img = np.reshape(img, (img.shape[0],img.shape[1],1))
        return img

    def array2string(self, array):
        if len(array) >= 1: 
            result = "\'  ,  \'".join(array)
            result = "[\'" + result
            result += "\']"
            return result
        else:
            return "[]"

    def getClassName(self, classNo):        
        if   classNo == 0: return 'stop sign'
        elif classNo == 1: return 'straight sign'
        elif classNo == 2: return 'roundobout'
        elif classNo == 3: return 'turn right ahead sign'
        elif classNo == 4: return 'turn left ahead sign'
        elif classNo == 5: return 'end of prhibition sign'
        """if   classNo == 0: return 'stop sign'
        elif classNo == 1: return 'straight sign'
        elif classNo == 2: return 'turn right ahead sign'"""        

    def imgmsg_to_cv2(self, ros_image):
        if ros_image.encoding == "bgr8" and ros_image.is_bigendian == 0:
            cv_img = np.array(list(ros_image.data), dtype= np.uint8)
            cv_img = np.reshape(cv_img, (ros_image.height, ros_image.step))
            cv_img = np.reshape(cv_img, (ros_image.height, ros_image.width, int(ros_image.step/ros_image.width) ) )
        else:
            cv_img = None
            raise Exception("Error while convering ros image message to a cv image")                  
        return cv_img

    def cv2_to_imgmsg(self, image, encoding = "bgr8"):
        #print("cv2_to_imgmsg image shape is:" + str(image.shape))
        if encoding == "bgr8":
            self.curr_signs_image_msg.header = Header()
            self.curr_signs_image_msg.height = image.shape[0]
            self.curr_signs_image_msg.width = image.shape[1]
            self.curr_signs_image_msg.encoding = encoding
            self.curr_signs_image_msg.is_bigendian = 0
            self.curr_signs_image_msg.step = image.shape[1]*image.shape[2]

            data = np.reshape(image, (self.curr_signs_image_msg.height, self.curr_signs_image_msg.step) )
            data = np.reshape(image, (self.curr_signs_image_msg.height*self.curr_signs_image_msg.step) )
            data = list(data)
            self.curr_signs_image_msg.data = data
            return self.curr_signs_image_msg
        else:            
            raise Exception("Error while convering cv image to ros message") 
            return None


    def main(self):
        while not rospy.is_shutdown():
            #print("starting main")
            if self.image != None:
                #print("image is different to none")                            
                self.cv_original_image = self.imgmsg_to_cv2(self.image)
                self.cv_original_image = cv.rotate(self.cv_original_image,cv.ROTATE_180)
                (originalWidth, originalHeight, _) = self.cv_original_image.shape

                if self.simulation:
                    self.newWidth = int((1.0/2.0)*(originalWidth))
                    self.newHeight = int((1.0/2.0)*(originalHeight))
                else:
                    self.newWidth = int((1.0/3.0)*(originalWidth))
                    self.newHeight = int((1.0/3.0)*(originalHeight))

                #print("new dimensions are: " + str((self.newHeight, self.newWidth)))
                #print("image is " + str(self.cv_original_image) )
                self.cv_image = cv.resize(self.cv_original_image,(self.newHeight,self.newWidth), interpolation = cv.INTER_AREA)
                self.final_detection = self.cv_image.copy()

                HSV = cv.cvtColor(self.cv_image, cv.COLOR_BGR2HSV)     
        
                red1 = cv.inRange(HSV,(145,70,50),(180,255,255))
                red2 = cv.inRange(HSV,(0,70,50),(10,255,255))
                red = cv.bitwise_or(red1,red2)  
                
                blue = cv.inRange(HSV,(85,70,50),(140,255,255)) 

                #black = cv.inRange(HSV,(0,0,0),(255,255,100))
                black = cv.cvtColor(self.cv_image, cv.COLOR_BGR2GRAY)
                _,black = cv.threshold(black, 128, 255, cv.THRESH_BINARY_INV)    
                black = cv.dilate(black, self.kernel, iterations=1) 

                """redHexagons = self.red_hexagons_classifier.detectMultiScale(red, scaleFactor=1.1, minNeighbors=350, minSize=(40,40))
                blueCircles = self.blue_circles_classifier.detectMultiScale(blue, scaleFactor=1.1, minNeighbors=700, minSize=(40,40))
                blackCircles = self.black_circles_classifier.detectMultiScale(black, scaleFactor=1.1, minNeighbors=300, minSize=(40,40))"""
                
                if self.simulation:
                    stopSignsHC = self.stopSignClassifier.detectMultiScale(red, scaleFactor=1.1, minNeighbors=100, minSize=(20,20))
                    goStraightSignsHC = self.goStraightClassifier.detectMultiScale(blue, scaleFactor=1.1, minNeighbors=50, minSize=(20,20))
                    turnRightAheadSignsHC = self.turnRightAheadClassifier.detectMultiScale(blue, scaleFactor=1.1, minNeighbors=150, minSize=(20,20))
                else:
                    stopSignsHC = self.stopSignClassifier.detectMultiScale(red, scaleFactor=1.1, minNeighbors=30, minSize=(10,10))
                    goStraightSignsHC = self.goStraightClassifier.detectMultiScale(blue, scaleFactor=1.1, minNeighbors=5, minSize=(10,10))
                    turnRightAheadSignsHC = self.turnRightAheadClassifier.detectMultiScale(blue, scaleFactor=1.1, minNeighbors=120, minSize=(10,10))


                for (x,y,w,h) in stopSignsHC:                                    
                    red_hexagon_roi = self.cv_image[y:y+h,x:x+w]
                    red_hexagon_roi = cv.resize(red_hexagon_roi,(45,45), interpolation = cv.INTER_AREA)   
                    red_hexagon_roi = self.cnn_preprocessing(red_hexagon_roi)
                    red_hexagon_roi = red_hexagon_roi.reshape(1, 45, 45, 1)
                    #predictions = self.cnn_model.predict(red_hexagon_roi)        
                    #classIndex = np.argmax(predictions)
                    #probabilityValue = np.amax(predictions)
                    #className = self.getClassName(classIndex)
                    cv.rectangle(self.final_detection, (x,y),(x+w,y+h),(0,0,255),2)
                    cv.putText(self.final_detection,'stop sign, prob=',(x,y-10),2,0.7,(0,0,255),2,cv.LINE_AA)
                    """
                    if probabilityValue > self.cnn_probability_threshold and className == "stop sign":
                        cv.rectangle(self.final_detection, (x,y),(x+w,y+h),(0,0,255),2)
                        cv.putText(self.final_detection,'{class_name}, prob={prob:.2f}'.format(class_name=className, prob=probabilityValue),(x,y-10),2,0.7,(0,0,255),2,cv.LINE_AA)
                        self.curr_traffic_signs.append(className)
                        """

                for (x,y,w,h) in goStraightSignsHC:
                    blue_circle_roi = self.cv_image[y:y+h,x:x+w]
                    blue_circle_roi = cv.resize(blue_circle_roi,(45,45), interpolation = cv.INTER_AREA)   
                    blue_circle_roi = self.cnn_preprocessing(blue_circle_roi)
                    blue_circle_roi = blue_circle_roi.reshape(1, 45, 45, 1)
                    #predictions = self.cnn_model.predict(blue_circle_roi)        
                    #classIndex = np.argmax(predictions)
                    #probabilityValue = np.amax(predictions)
                    #className = self.getClassName(classIndex)
                    cv.rectangle(self.final_detection, (x,y),(x+w,y+h),(255,0,0),2)
                    cv.putText(self.final_detection,'go straight, prob = ',(x,y-10),2,0.7,(0,0,255),2,cv.LINE_AA)
                    """
                    if probabilityValue > self.cnn_probability_threshold and (className == "straight sign"):
                        cv.rectangle(self.final_detection, (x,y),(x+w,y+h),(255,0,0),2)
                        cv.putText(self.final_detection,'{class_name}, prob={prob:.2f}'.format(class_name=className, prob=probabilityValue),(x,y-10),2,0.7,(255,0,0),2,cv.LINE_AA)
                        self.curr_traffic_signs.append(className)
                        """

                for (x,y,w,h) in turnRightAheadSignsHC:
                    blue_circle_roi = self.cv_image[y:y+h,x:x+w]
                    blue_circle_roi = cv.resize(blue_circle_roi,(45,45), interpolation = cv.INTER_AREA)   
                    blue_circle_roi = self.cnn_preprocessing(blue_circle_roi)
                    blue_circle_roi = blue_circle_roi.reshape(1, 45, 45, 1)
                    #predictions = self.cnn_model.predict(blue_circle_roi)
                    #classIndex = np.argmax(predictions)
                    #probabilityValue = np.amax(predictions)
                    #className = self.getClassName(classIndex)
                    cv.rectangle(self.final_detection, (x,y),(x+w,y+h),(255,0,0),2)
                    cv.putText(self.final_detection,'turn right ahead, prob=',(x,y-10),2,0.7,(0,0,255),2,cv.LINE_AA)
                    """
                    if probabilityValue > self.cnn_probability_threshold and (className == "turn right ahead sign"):
                        cv.rectangle(self.final_detection, (x,y),(x+w,y+h),(255,0,0),2)
                        cv.putText(self.final_detection,'{class_name}, prob={prob:.2f}'.format(class_name=className, prob=probabilityValue),(x,y-10),2,0.7,(255,0,0),2,cv.LINE_AA)
                        self.curr_traffic_signs.append(className)
                        """

                self.curr_signs_msg.data = self.array2string(self.curr_traffic_signs)
                self.pub_curr_signs.publish(self.curr_signs_msg)
                self.curr_traffic_signs = []
                self.curr_signs_image_msg = self.cv2_to_imgmsg(self.final_detection, encoding = "bgr8")
                self.pub_curr_signs_image.publish(self.curr_signs_image_msg)
            bk.clear_session()
            self.rate.sleep()

#Si el archivo es corrido directametne y no llamado desde otro archivo corremos
if __name__ == "__main__":
    
    bk.clear_session()
    #  ==================== we extract the ROS arguments passed to init the node =======================
    arguments  = rospy.myargv(argv=sys.argv)
    if len(arguments) > 1:
        if arguments[1] == "simulation":
            sim = True
            camera_topic = "/camera/image_rotated"
        else:
            sim = False
            camera_topic = "/video_source/raw"
    else:
        sim = False
        camera_topic = "/video_source/raw"
    print("initial arguments are:")
    print("camera topic = " + camera_topic)
        
    #  ==================== ================================================== =======================

    detector = trafficSignalsDetector(cam_topic = camera_topic, simulation = sim)
    detector.main()

            
    #mov.main()
