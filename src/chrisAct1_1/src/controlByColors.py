#!/usr/bin/env python
# coding=utf-8
import numpy as np
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class controlColors():
    def __init__(self):
        #Inicializamos el nodo
        rospy.init_node("controlByColors")  
        #Creamos el publisher

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        #Creamos los susbcribers
        self.circleRed = rospy.Subscriber("/processedImage/detectedObjects/redCircle",Bool,self.red_circle_callback)
        self.circleGreen = rospy.Subscriber("/processedImage/detectedObjects/greenCircle",Bool,self.green_circle_callback)
        self.circleYellow = rospy.Subscriber("/processedImage/detectedObjects/yellowCircle",Bool,self.yellow_circle_callback)

        self.redCircleDetected = None
        self.greenCircleDetected = None
        self.yellowCircleDetected = None

        self.msg = Twist()

        self.maxVel = 0.5

        self.rate = rospy.Rate(10)
    
    def red_circle_callback(self, data):
    	self.redCircleDetected = data.data

    def green_circle_callback(self, data):
    	self.greenCircleDetected = data.data

    def yellow_circle_callback(self, image):
    	self.yellowCircleDetected = data.data

    def move(self,v_lin,v_ang):
        """Funcion que publica la velocidad lineal y angular en el puzzlebot"""
        #Declaramos le velocidad deseada en el mensaje tipo Twist
        print("Avanzando a",v_lin,"m/s")
        self.msg.linear.x = v_lin
        self.msg.linear.y = 0.0
        self.msg.linear.z = 0.0
        self.msg.angular.x = 0.0
        self.msg.angular.y = 0.0
        self.msg.angular.z = v_ang
        #Publicamos la velocidad
        self.pub.publish(self.msg)

    def main(self):
    	while not rospy.is_shutdown():
    		if (self.redCircleDetected != None) and (self.greenCircleDetected != None) and (self.yellowCircleDetected != None):
    			if (self.redCircleDetected == True):
    				self.move(0.0,0.0)
                elif (self.yellowCircleDetected == True):
                    self.move(self.maxVel/2,0.0)
                else:
                    self.move(self.maxVel,0.0)
                self.rate.sleep()


if __name__ == "__main__":
	controlC = controlColors()
	controlC.main()

