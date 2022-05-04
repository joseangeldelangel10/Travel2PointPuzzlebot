#!/usr/bin/env python
# coding=utf-8
import sys
import numpy as np
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Pose2D
#Creamos la clase
class MovePuzzlebot():
    def __init__(self):
        #Inicializamos el nodo
        rospy.init_node("circulo_mov_traffic_lights")
        #Creamos el publisher
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pubTargetBool = rospy.Publisher("/arrive2Target", Bool, queue_size=1)
        #Creamos los subscribers

        self.circleRed = rospy.Subscriber("/processedImage/detectedObjects/redCircle",Bool,self.red_circle_callback)
        self.circleGreen = rospy.Subscriber("/processedImage/detectedObjects/greenCircle",Bool,self.green_circle_callback)
        self.circleYellow = rospy.Subscriber("/processedImage/detectedObjects/yellowCircle",Bool,self.yellow_circle_callback)

        self.subPose = rospy.Subscriber("/pose2d",Pose2D,self.odom_callback)

        self.subTarget = rospy.Subscriber("/target",Pose2D,self.target_callback)

        #Declaramos que vamos a mandar 20 mensajes por segundo.
        self.rate = rospy.Rate(100)
        #Creamos el msg Twist
        self.msg = Twist()

        self.redCircleDetected = None
        self.greenCircleDetected = None
        self.yellowCircleDetected = None

        self.current_pose = None

        self.target = None
        
        #self.theta_target = None
        
        #self.theta_reached = False
        #self.pos_reached = False
        self.state = "travelingTowardsGoal"
        self.colorState = "Stopped"
        #Creamos un función de que hacer cuando haya un shutdown
        rospy.on_shutdown(self.end_callback)


    def red_circle_callback(self, data):
        self.redCircleDetected = data.data

    def green_circle_callback(self, data):
        self.greenCircleDetected = data.data

    def yellow_circle_callback(self, data):
        self.yellowCircleDetected = data.data

    def odom_callback(self, odom):
        self.current_pose = odom

    def target_callback(self, pose):
        self.target = [pose.x,pose.y]

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

    def sendBool(self,boolean):
        result = Bool()
        result.data = boolean
        self.pubTargetBool.publish(result)

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
    #mientras este corriendo el nodo movemos el carro el circulo
    Kpw = 0.92
    Kpv = 0.25
    while not rospy.is_shutdown():
        #Llamamos el sleep para asegurar los 20 msg por segundo
        
        if mov.current_pose != None and mov.target != None:
            #print("None condition passed")

            theta_target = np.arctan2(mov.target[1]-mov.current_pose.y, mov.target[0]-mov.current_pose.x)
            e_theta = theta_target - mov.current_pose.theta
            e_pos = np.sqrt((mov.target[0]-mov.current_pose.x)**2 + (mov.target[1]-mov.current_pose.y)**2)
            
            if mov.state == "travelingTowardsGoal":
                w = Kpw*e_theta
                v = Kpv*e_pos

                           
                if w >= 0.2:
                    w = 0.19
                elif w <= -0.2:
                    w = -0.19
                if v >= 0.2:                   
                    v = 0.19                                        
                elif v <= -0.2:                    
                    v = -0.19

                if (mov.redCircleDetected == True):
                    mov.colorState = "Stopped"
                if (mov.yellowCircleDetected == True):
                    mov.colorState = "Running Half"
                if (mov.greenCircleDetected == True):
                    mov.colorState = "Running"

                if (mov.colorState == "Running Half"):
                    v = v/2
                    w = w/2
                elif (mov.colorState == "Stopped"):
                    v = 0.0
                    w = 0.0                
                
                mov.move(v,w)
                mov.sendBool(False)
                

                
            if (abs(e_theta) <= 0.025) and (abs(e_pos) <= 0.02 and mov.state == "travelingTowardsGoal"):
                mov.move(0.0,0.0)
                mov.state = "goalReached"
                print("============ final pose based in odometry is: =============")
                print(mov.current_pose)
                mov.sendBool(True)

            if mov.state == "goalReached":
                mov.move(0.0,0.0)
                mov.sendBool(False) 

            if (abs(e_theta) >= 0.025) and (abs(e_pos) >= 0.02 and mov.state == "goalReached"):
                print("NEW TARGET RECIEVED AND STATE CHANGED")
                mov.state = "travelingTowardsGoal"
                mov.sendBool(False)


            
        mov.rate.sleep()

            
    #mov.main()
