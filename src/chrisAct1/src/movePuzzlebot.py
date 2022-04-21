#!/usr/bin/env python
# coding=utf-8
import sys
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
#Creamos la clase
class MovePuzzlebot():
    def __init__(self):
        #Inicializamos el nodo
        rospy.init_node("circulo_mov")
        #Creamos el publisher
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/odometry",Odometry,self.odom_callback)
        #Declaramos que vamos a mandar 20 mensajes por segundo.
        self.rate = rospy.Rate(3000)
        #Creamos el msg Twist
        self.msg = Twist()

        self.current_pose = None

        self.target = [0.0,1.0]
        
        self.theta_target = np.atan2(self.target[0], self.target[1])
        
        self.theta_reached = False
        self.pos_reached = False
        #Creamos un función de que hacer cuando haya un shutdown
        rospy.on_shutdown(self.end_callback)
    def odom_callback(self, odom):
        self.current_pose = odom.pose.pose
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
#Si el archivo es corrido directametne y no llamado desde otro archivo corremos
if __name__ == "__main__":
    #iniciamos la clase
    mov = MovimientoCirculo()
    #mientras este corriendo el nodo movemos el carro el circulo
    while not rospy.is_shutdown():
        #Llamamos el sleep para asegurar los 20 msg por segundo
        mov.rate.sleep()
        if self.current_pose != None
            e_theta = self.theta_target - self.current_pose.orientation.z
            e_pos = np.sqrt((self.target[0]-self.current_pose.position.x)**2 + (self.target[1]-self.current_pose.position.y)**2)
            if !self.theta_reached and abs(e_theta) > 0.1 :
                mov.move(0,-0.1*np.sign(e_theta))
            elif abs(e_theta) < 0.1:
                self.theta_reached = True
                mov.move(0.0,0.0)
            if self.theta_reached and !self.pos_reached and abs(e_pos) > 0.1:
                mov.move(0,0.1*np.sign(e_pos))
            elif abs(e_pos) < 0.1:
                self.pos_reached = True
                mov.move(0.0,0.0)

            print("============ final pose based in odometry is: =============")
            print(self.current_pose)
    #mov.main()
