#!/usr/bin/env python
# coding=utf-8
import sys
import numpy as np
import rospy
from std_msgs.msg import Bool String
from geometry_msgs.msg import Twist, Pose2D
#Creamos la clase
class MovePuzzlebot():
    def __init__(self):
        #Inicializamos el nodo
        rospy.init_node("goToPoints")
        #Creamos el publisher
        self.pub = rospy.Publisher("/cmd_vel_g2p", Twist, queue_size=1)

        self.successPub = rospy.Publisher("/g2p_success", Bool, queue_size = 1)
        
        self.subPose = rospy.Subscriber("/pose2d",Pose2D,self.odom_callback)

        self.subMode = rospy.Subscriber("/g2p_mode", String, self.mode_callback)

        self.subCurrAction = rospy.Subscriber("/curr_action", String, self.curr_action_callback)

        #Declaramos que vamos a mandar 20 mensajes por segundo.
        self.rate = rospy.Rate(100)
        self.msg = Twist()

        self.current_pose = None

        self.g2p_mode = None

        self.target_index = 0

        self.targets = None

        self.curr_action = None
        self.success = True 

        self.go_straigt_targets = [[0.5,0.0]]
        self.turn_right_targets = [[0.5,0.0],[0.5,0.5]]

        self.current_target = None
        
        self.state = "travelingTowardsGoal"

        #Creamos un función de que hacer cuando haya un shutdown
        rospy.on_shutdown(self.end_callback)

    def odom_callback(self, odom):
        self.current_pose = odom

    def mode_callback(self, data_string):
        if self.success:
            self.g2p_mode = data_string.data
            if self.g2p_mode == "go straight":
                self.targets = self.go_straigt_targets
            elif self.g2p_mode == "turn right ahead":
                self.targets = self.turn_right_targets

            self.current_target = self.targets[self.target_index]

    def curr_action_callback(self, curr_action):
            self.curr_action = curr_action.data

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

    def sendSuccessValue(self,value):
        msg = Bool()
        msg.data = value
        self.successPub.publish(msg)


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
    Kpw = 1.0
    Kpv = 0.25
    while not rospy.is_shutdown():
        #Llamamos el sleep para asegurar los 20 msg por segundo
        
        if mov.current_pose != None and mov.curr_action == "go to point":
            #print("None condition passed")
 

            theta_target = np.arctan2(mov.current_target[1]-mov.current_pose.y, mov.current_target[0]-mov.current_pose.x)
            e_theta = theta_target - mov.current_pose.theta
            e_pos = np.sqrt((mov.current_target[0]-mov.current_pose.x)**2 + (mov.current_target[1]-mov.current_pose.y)**2)
            
            if mov.state == "pointingTowardsGoal":
                mov.sendSuccessValue(False)
                w = Kpw*e_theta
                v = 0.0

                           
                if w >= 0.25:
                    w = 0.24
                elif w <= -0.25:
                    w = -0.24               

                mov.move(v,w)

            if mov.state == "travelingTowardsGoal":
                mov.sendSuccessValue(False)
                w = Kpw*e_theta
                v = Kpv*e_pos

                           
                if w >= 0.25:
                    w = 0.24
                elif w <= -0.25:
                    w = -0.24
                if v >= 0.25:                   
                    v = 0.24                                        
                elif v <= -0.25:                    
                    v = -0.24                
                
                mov.move(v,w)
        
            if (abs(e_theta) <= 0.025) and mov.state == "pointingTowardsGoal":
                mov.sendSuccessValue(False)
                mov.state = "travelingTowardsGoal"
                    
            if (abs(e_theta) <= 0.025) and (abs(e_pos) <= 0.02 and mov.state == "travelingTowardsGoal"):
                mov.sendSuccessValue(False)
                mov.move(0.0,0.0)
                print("============ target {target_num} reached, pose based in odometry is: =============".format(target_num = mov.target_index))
                print(mov.current_pose)
                if mov.target_index == len(mov.targets)-1:
                    mov.state = "goalReached"
                else:                    
                    mov.target_index += 1
                    mov.current_target = mov.targets[mov.target_index]
                    mov.state = "pointingTowardsGoal"
                
            if mov.state == "goalReached":
                mov.sendSuccessValue(True)
                mov.move(0.0,0.0)
                
        mov.rate.sleep()