#!/usr/bin/env python
# coding=utf-8
import sys
import numpy as np
import rospy
from geometry_msgs.msg import Twist, Pose2D
#Creamos la clase
class MovePuzzlebot():
    def __init__(self):
        #Inicializamos el nodo
        rospy.init_node("circulo_mov_PID")
        #Creamos el publisher
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/pose2d",Pose2D,self.odom_callback)
        #Declaramos que vamos a mandar 20 mensajes por segundo.
        self.rate = rospy.Rate(300)
        #Creamos el msg Twist
        self.msg = Twist()

        self.current_pose = None

        self.target = [1.0,1.0]
        
        self.theta_target = np.arctan2(self.target[1], self.target[0])
        
        #self.theta_reached = False
        #self.pos_reached = False
        self.state = "travelingTowardsGoal"
        #Creamos un función de que hacer cuando haya un shutdown
        rospy.on_shutdown(self.end_callback)
    def odom_callback(self, odom):
        self.current_pose = odom
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
    mov = MovePuzzlebot()
    e_int_pos = 0
    e_int_theta = 0
    last_time = rospy.get_time()
    #mientras este corriendo el nodo movemos el carro el circulo
    while not rospy.is_shutdown():
        #Llamamos el sleep para asegurar los 20 msg por segundo
        mov.rate.sleep()
        if mov.current_pose != None:
            e_theta = mov.theta_target - mov.current_pose.theta
            e_pos = np.sqrt((mov.target[0]-mov.current_pose.x)**2 + (mov.target[1]-mov.current_pose.y)**2)
            current_time = rospy.get_time()
            dt = current_time - last_time
            if dt > 0.1:
                dt = 0.003333
            e_int_theta += e_theta*dt
            e_int_pos += e_pos*dt
            Kiw = 0.25
            Kiv = 0.125
            Kpw = 0.1
            Kpv = 0.1
            w = Kpw*e_theta + Kiw*e_int_theta
            #v = Kpv*e_pos + Kiv*e_int_pos
            v = Kpv*e_pos
            last_time = current_time
            if mov.state == "travelingTowardsGoal":
                if w < 0.7 or v < 0.7: 
                    mov.move(v,w)
                else:
                    w = 0.69
                    v = 0.69
                    mov.move(v,w)

            if (abs(e_theta) <= 0.03) and (abs(e_pos) <= 0.01 and mov.state == "travelingTowardsGoal"):
                mov.move(0.0,0.0)
                mov.state = "goalReached"
                print("============ final pose based in odometry is: =============")
                print(mov.current_pose)
                print("============ dt is: =============")
                print(dt)
                mov.end_callback()            


            
    #mov.main()
