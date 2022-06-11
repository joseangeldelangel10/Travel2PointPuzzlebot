#!/usr/bin/env python3
# coding=utf-8
import cv2 as cv
import sys
import numpy as np
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

        self.class_square_size = (20,20)
        self.class_k_height = 20
        self.class_k_width = 20
        self.class_kernel = np.ones((20,20))
        self.class_small_kernel = np.ones((2,2), np.uint8)

        self.class_kernel_A, self.class_kernel_B, self.class_kernel_C = self.define_classifier_kernels() 
        
        #self.bridge = cv_bridge.CvBridge()

        #Declaramos que vamos a mandar 20 mensajes por segundo.        

        self.rate = rospy.Rate(5)
        self.rateInt = 5
        print("end of init reached")                                              

    def on_image_callback(self, image):
        self.image = image  

    def cnn_preprocessing(self, img):        
        img = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
        _,img = cv.threshold(img, 128, 255, cv.THRESH_BINARY)
        img = cv.dilate(img, self.class_small_kernel, iterations=1)
        #img = np.reshape(img, (img.shape[0],img.shape[1],1))
        return img

    def array2string(self, array):
        if len(array) >= 1: 
            result = "\'  ,  \'".join(array)
            result = "[\'" + result
            result += "\']"
            return result
        else:
            return "[]"

    def rotate_image(self, image, angle):
      image_center = tuple(np.array(image.shape[1::-1]) / 2)
      rot_mat = cv.getRotationMatrix2D(image_center, angle, 1.0)
      result = cv.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv.INTER_LINEAR)
      return result

    def define_classifier_kernels(self):
        # =========== KERNEL DEFINITIONS ==============
        kernel_A = self.class_kernel.copy()
        kernel_A[0:int(1/4*self.class_k_height),:] = self.class_kernel[0:int(1/4*self.class_k_height),:]*(-255)
        kernel_A[int(1/4*self.class_k_height):int(3/4*self.class_k_height),:] = self.class_kernel[int(1/4*self.class_k_height):int(3/4*self.class_k_height),:]*255
        kernel_A[int(3/4*self.class_k_height):,:] = self.class_kernel[int(3/4*self.class_k_height):,:]*(-255)
        kernel_A = kernel_A/255.0
        #print(kernel_A)

        kernel_B = self.class_kernel.copy()
        kernel_B[:,0:int(1/4*self.class_k_width)] = self.class_kernel[:,0:int(1/4*self.class_k_width)]*(-255)
        kernel_B[:,int(1/4*self.class_k_width):int(3/4*self.class_k_width)] = self.class_kernel[:,int(1/4*self.class_k_width):int(3/4*self.class_k_width)]*255
        kernel_B[:,int(3/4*self.class_k_width):] = self.class_kernel[:,int(3/4*self.class_k_width):]*(-255)
        kernel_B = kernel_B/255.0

        kernel_C = self.class_kernel.copy()
        kernel_C[:,:] = self.class_kernel[:,:]*255
        kernel_C[int(1/2*self.class_k_width):,int(1/2*self.class_k_width):] = self.class_kernel[int(1/2*self.class_k_width):,int(1/2*self.class_k_width):]*(-255)
        kernel_C[:3,:] = self.class_kernel[:3,:]*(-255)
        kernel_C[:,:3] = self.class_kernel[:,:3]*(-255)
        kernel_C = kernel_C/255.0

        hexagon_kernel = self.class_kernel.copy()
        hexagon_kernel = np.uint8(hexagon_kernel)
        hexagon_kernel[:,:] = self.class_kernel[:,:]*(255)
        hexagon_kernel = self.rotate_image(hexagon_kernel, 45)
        _,hexagon_kernel = cv.threshold(hexagon_kernel, 128, 255, cv.THRESH_BINARY)
        hexagon_kernel = np.logical_not(hexagon_kernel) * (255.0)
        hexagon_kernel = np.where(hexagon_kernel == 0.0,  -255.0, 255.0)
        hexagon_kernel = hexagon_kernel/255.0

        kernel_A = np.maximum(kernel_A, hexagon_kernel)
        kernel_B = np.maximum(kernel_B, hexagon_kernel)
        kernel_C = np.maximum(kernel_C, hexagon_kernel)

        return (kernel_A, kernel_B, kernel_C)

    def classify(self, image):
        objeto = image
        #print(objeto.shape)
        data_a = objeto*self.class_kernel_A
        data_a = data_a.sum()
        data_b = objeto*self.class_kernel_B
        data_b = data_b.sum()
        data_c = objeto*self.class_kernel_C
        data_c = data_c.sum()

        return np.array([data_a, data_b, data_c])      

    def getClassName(self, classNo):        
        """if   classNo == 0: return 'stop sign'
        elif classNo == 1: return 'straight sign'
        elif classNo == 2: return 'roundobout'
        elif classNo == 3: return 'turn right ahead sign'
        elif classNo == 4: return 'turn left ahead sign'
        elif classNo == 5: return 'end of prhibition sign'"""
        if   classNo == 0: return 'stop sign'
        elif classNo == 1: return 'straight sign'
        elif classNo == 2: return 'turn right ahead sign'        

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
                    self.newWidth = int((1.0/2.0)*(originalWidth))
                    self.newHeight = int((1.0/2.0)*(originalHeight))

                #print("new dimensions are: " + str((self.newHeight, self.newWidth)))
                #print("image is " + str(self.cv_original_image) )
                self.cv_image = cv.resize(self.cv_original_image,(self.newHeight,self.newWidth), interpolation = cv.INTER_AREA)
                self.final_detection = self.cv_image.copy()

                HSV = cv.cvtColor(self.cv_image, cv.COLOR_BGR2HSV)     
        
                red1 = cv.inRange(HSV,(145,50,50),(180,255,255))
                red2 = cv.inRange(HSV,(0,50,50),(10,255,255))
                red = cv.bitwise_or(red1,red2)  
                
                blue = cv.inRange(HSV,(85,50,50),(140,255,255))                
                
                redHexagonsCotours, _ = cv.findContours(red, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
                blueCirclesCotours, _ = cv.findContours(blue, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
                #self.outImage = cv.merge((self.binary_image, self.binary_image, self.binary_image))

                #print(contours)

                red_hexagons = []
                for c in redHexagonsCotours:
                    (min_circle_x,min_circle_y),min_circle_radius = cv.minEnclosingCircle(c)
                    bounding_box = cv.minAreaRect(c)
                    bounding_box_x = bounding_box[1][0]
                    bounding_box_y = bounding_box[1][1]
                    if abs((min_circle_radius*2)-bounding_box_x) <= min_circle_radius*0.4 and abs((min_circle_radius*2)-bounding_box_y) <= min_circle_radius*0.4:
                        square_area = bounding_box_x*bounding_box_y
                        circle_area = np.pi*(min_circle_radius**2)
                        if circle_area <= square_area:
                            cords = cv.boundingRect(c)
                            red_hexagons.append( cords )

                blueCircles = []
                for c in blueCirclesCotours:
                    (min_circle_x,min_circle_y),min_circle_radius = cv.minEnclosingCircle(c)
                    bounding_box = cv.minAreaRect(c)
                    bounding_box_x = bounding_box[1][0]
                    bounding_box_y = bounding_box[1][1]
                    if abs((min_circle_radius*2)-bounding_box_x) <= min_circle_radius*0.35 and abs((min_circle_radius*2)-bounding_box_y) <= min_circle_radius*0.35:
                        square_area = bounding_box_x*bounding_box_y
                        circle_area = np.pi*(min_circle_radius**2)
                        if circle_area <= square_area:
                            cords = cv.boundingRect(c)
                            blueCircles.append( cords )                    

                for (x,y,w,h) in red_hexagons:                                    
                    red_hexagon_roi = self.cv_image[y:y+h,x:x+w]
                    red_hexagon_roi = cv.resize(red_hexagon_roi,self.class_square_size, interpolation = cv.INTER_AREA)   
                    red_hexagon_roi = self.cnn_preprocessing(red_hexagon_roi)                    
                    predictions = self.classify(red_hexagon_roi)        
                    classIndex = np.argmax(predictions)
                    probabilityValue = np.amax(predictions)
                    className = self.getClassName(classIndex)
                    #cv.rectangle(self.final_detection, (x,y),(x+w,y+h),(0,0,255),2)
                    #cv.putText(self.final_detection,'stop sign, prob=',(x,y-10),2,0.7,(0,0,255),2,cv.LINE_AA)                    
                    if probabilityValue > self.cnn_probability_threshold and className == "stop sign":
                        cv.rectangle(self.final_detection, (x,y),(x+w,y+h),(0,0,255),2)
                        cv.putText(self.final_detection,'{class_name}, prob={prob:.2f}'.format(class_name=className, prob=probabilityValue),(x,y+h),2,0.7,(255,0,0),2,cv.LINE_AA)
                        self.curr_traffic_signs.append(className)
                        

                for (x,y,w,h) in blueCircles:
                    blue_circle_roi = self.cv_image[y:y+h,x:x+w]
                    blue_circle_roi = cv.resize(blue_circle_roi,self.class_square_size, interpolation = cv.INTER_AREA)   
                    blue_circle_roi = self.cnn_preprocessing(blue_circle_roi)                    
                    predictions = self.classify(blue_circle_roi)        
                    classIndex = np.argmax(predictions)
                    probabilityValue = np.amax(predictions)
                    className = self.getClassName(classIndex)
                    #cv.rectangle(self.final_detection, (x,y),(x+w,y+h),(255,0,0),2)
                    #cv.putText(self.final_detection,'go straight, prob = ',(x,y-10),2,0.7,(0,0,255),2,cv.LINE_AA)                    
                    if probabilityValue > self.cnn_probability_threshold and (className == "straight sign" or className == "turn right ahead sign"):
                        cv.rectangle(self.final_detection, (x,y),(x+w,y+h),(255,0,0),2)
                        cv.putText(self.final_detection,'{class_name}, prob={prob:.2f}'.format(class_name=className, prob=probabilityValue),(x,y+h),2,0.7,(255,0,0),2,cv.LINE_AA)
                        self.curr_traffic_signs.append(className)
                        

                self.curr_signs_msg.data = self.array2string(self.curr_traffic_signs)
                self.pub_curr_signs.publish(self.curr_signs_msg)
                self.curr_traffic_signs = []
                self.curr_signs_image_msg = self.cv2_to_imgmsg(self.final_detection, encoding = "bgr8")
                self.pub_curr_signs_image.publish(self.curr_signs_image_msg)
            self.rate.sleep()

#Si el archivo es corrido directametne y no llamado desde otro archivo corremos
if __name__ == "__main__":
    
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
