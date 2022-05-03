#!/usr/bin/env python
# coding=utf-8
import sys
import numpy as np
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Pose2D

class TargetSelector():
    def __init__(self):
        #Inicializamos el nodo
        rospy.init_node("TargetSelector")
        #Creamos el publisher
        self.pub = rospy.Publisher("/target", Pose2D, queue_size=1)
        #Creamos los subscribers

        self.subscriber = rospy.Subscriber("/arrive2Target",Bool,self.arrived2Target_callback)
       

        #Declaramos que vamos a mandar 20 mensajes por segundo.
        self.rate = rospy.Rate(100)
        #Creamos el msg Twist
        self.msg = Pose2D()

        self.arrived2Target = None

        self.current_pose = None


        self.targets = [[2.0,0.0],[2.0,2.0],[0.0,2.0],[0.0,0.0]]

        self.target_index = 0

        self.current_target = self.targets[self.target_index]
        
        


    def arrived2Target_callback(self, data):
        self.arrived2Target = data.data

if __name__ == "__main__":
    #iniciamos la clase
    tar = TargetSelector()
   
    while not rospy.is_shutdown():
        #Llamamos el sleep para asegurar los 20 msg por segundo
        
        if tar.arrived2Target != None:

			if tar.arrived2Target == True:
				if tar.target_index == len(tar.targets)-1:
					break
        		tar.target_index += 1
        		tar.current_target = tar.targets[tar.target_index]


        	tar.msg.x = tar.current_target[0]
        	tar.msg.y = tar.current_target[1]
        	tar.pub.publish(tar.msg)



            
