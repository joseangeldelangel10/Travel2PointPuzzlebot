#!/usr/bin/env python
# coding=utf-8
import sys
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2 as cv
import cv_bridge
import tensorflow as tf
from tensorflow.keras.models import load_model
#from roseus.msg import StringStamped

#Creamos la clase
class trafficSignalsDetector():
    def __init__(self):
        #Inicializamos el nodo
        rospy.init_node("signalsDetector")        
        #Creamos el publisher
        self.pub_curr_signs = rospy.Publisher("/curr_traffic_signs", String, queue_size=1)
        
        #Creamos los subscribers
        self.sub_image = rospy.Subscriber("/video_source/raw",String, self.on_image_callback)
        
        self.curr_traffic_signs = []
        self.kernel = np.ones((5,5), np.uint8)
        self.red_hexagons_classifier = cv.CascadeClassifier("../../challenge_autonomous_nav/ai_models/haar/redHexagons.xml")
        self.blue_circles_classifier = cv.CascadeClassifier("../../challenge_autonomous_nav/ai_models/haar/blueCircles.xml")
        self.black_circles_classifier = cv.CascadeClassifier("../../challenge_autonomous_nav/ai_models/haar/blackCircles.xml")
        self.cnn_model = load_model('"../../challenge_autonomous_nav/ai_models/cnns/traffic_signs_cnn_model_5/my_model')
        self.cnn_probability_threshold = 0.90

        self.curr_signs_msg = String()

        #Declaramos que vamos a mandar 20 mensajes por segundo.
        self.rate = rospy.Rate(20)
        self.rateInt = 20                          
                    
        rospy.on_shutdown(self.end_callback)

    def on_last_iois_callback(self, data):
        self.last_iois = eval(data.data)        

    def on_curr_traffic_light(self, data):
        self.curr_traffic_lights = eval(data.data)        

    def on_curr_traffic_signs(self, data):
        self.curr_traffic_signs = eval(data.data)

    def end_callback(self):
        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_msg.angular.z = 0.0        
        self.pub_cmd_vel.publish(self.cmd_vel_msg)

    def publish_current_ioi(self, string):
        self.curr_ioi_msg.data = string        
        self.pub_curr_ioi.publish(self.curr_ioi_msg)

    def prorize_iois(self, iois):
        self.prorize_result = []
        if "straight sign" in iois:
            self.prorize_result.append("straight sign")
        if "turn right ahead sign" in iois:
            self.prorize_result.append("turn right ahead sign")
        if  "red light" in iois or "yellow light" in iois or "green light" in iois:
            if "red light" in iois:
                self.prorize_result.append("red light")
            elif "yellow light" in iois:
                self.prorize_result.append("yellow light")
            elif "green light" in iois:
                self.prorize_result.append("green light")
        if "stop sign" in iois:
            self.prorize_result.append("stop sign")
        if "end of prohibition sign" in iois:
            self.prorize_result.append("end of prohibition sign")        
        return self.prorize_result
        
    def compare_curr_iois_with_last_iois(self,curr, last):
        self.compare_result = [i for i in curr if i not in last]
        # we remove iois in current iois that are already in last iois
        return self.compare_result


    def main(self):
        while not rospy.is_shutdown():
            self.curr_iois = self.curr_traffic_lights + self.curr_traffic_signs            
            self.compare_result = self.compare_curr_iois_with_last_iois(self.curr_iois, self.last_iois)
            self.prorize_result = self.prorize_iois(self.compare_result)

            if len(self.prorize_result) > 0:
                self.curr_ioi = self.prorize_result[0]
            else:
                self.curr_ioi = "none"

            self.publish_current_ioi(self.curr_ioi)            
            
            self.rate.sleep()

#Si el archivo es corrido directametne y no llamado desde otro archivo corremos
if __name__ == "__main__":
    checker = iois_checker()
    checker.main()

            
    #mov.main()
