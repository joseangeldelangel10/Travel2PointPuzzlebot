#!/usr/bin/env python
# coding=utf-8
import sys
import numpy as np
import rospy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist, Pose2D
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
#Creamos la clase
class MovePuzzlebot():
    def __init__(self):
        #Inicializamos el nodo
        rospy.init_node("goToPoints")
        #Creamos el publisher
        self.pub = rospy.Publisher("/cmd_vel_g2p", Twist, queue_size=1)        
        
        self.subPose = rospy.Subscriber("/pose2d",Pose2D,self.odom_callback)
        #self.subMode = rospy.Subscriber("/g2p_mode", String, self.mode_callback)
        self.subCurrAction = rospy.Subscriber("/curr_action", String, self.curr_action_callback)

        #Declaramos que vamos a mandar 20 mensajes por segundo.
        self.rate = rospy.Rate(20)
        self.msg = Twist()

        self.current_pose = None
        #self.g2p_mode = None
        self.target_index = 0
        self.targets = None
        self.curr_action = None
        self.initializing = True        

        self.go_straigt_targets = [[0.7,0.0]]
        self.turn_right_targets = [[0.39,0.0],[0.356,-0.3]]
        #self.turn_right_targets = [[0.33,-0.03],[0.36,-0-36]]


        self.current_target = None
        
        self.state = "pointingTowardsGoal"

        rospy.wait_for_service("end_g2ps")
        try:
            self.end_g2ps = rospy.ServiceProxy("end_g2ps", Empty)
        except rospy.ServiceException as e:
            print(e)  

        #Creamos un función de que hacer cuando haya un shutdown
        rospy.on_shutdown(self.end_callback)

    def reset(self):        
        #self.current_pose = None
        #self.g2p_mode = None
        self.target_index = 0
        self.targets = None
        self.curr_action = None
        self.initializing = True        
        self.current_target = None        
        self.state = "pointingTowardsGoal"
        mov.move(0.0,0.0)

    def odom_callback(self, odom):
        self.current_pose = odom

    """def mode_callback(self, data_string):
        if self.initializing and self.curr_action=="go to point":
            self.g2p_mode = data_string.data
            if self.g2p_mode == "go straight":
                self.targets = self.go_straigt_targets
            elif self.g2p_mode == "turn right ahead":
                self.targets = self.turn_right_targets
            self.initializing = False
            self.target_index = 0
            self.current_target = self.targets[self.target_index]"""

    def curr_action_callback(self, data_string):
        if self.initializing:
            self.curr_action = data_string.data
            if self.curr_action == "go straight":
                self.targets = self.go_straigt_targets
                self.initializing = False
                self.target_index = 0
                self.current_target = self.targets[self.target_index]
            elif self.curr_action == "turn right ahead":
                self.targets = self.turn_right_targets
                self.initializing = False
                self.target_index = 0
                self.current_target = self.targets[self.target_index]        


    def move(self,v_lin,v_ang):
        """Funcion que publica la velocidad lineal y angular en el puzzlebot"""
        #Declaramos le velocidad deseada en el mensaje tipo Twist
        self.msg.linear.x = v_lin        
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
    #mientras este corriendo el nodo movemos el carro el circulo
    Kpw = 1.0
    Kpv = 0.25
    while not rospy.is_shutdown():        
        try:
            if mov.current_pose != None and (mov.curr_action == "go straight" or mov.curr_action == "turn right ahead"):
                #print("None condition passed")
     

                theta_target = np.arctan2(mov.current_target[1]-mov.current_pose.y, mov.current_target[0]-mov.current_pose.x)
                e_theta = theta_target - mov.current_pose.theta
                e_pos = np.sqrt((mov.current_target[0]-mov.current_pose.x)**2 + (mov.current_target[1]-mov.current_pose.y)**2)
                
                if mov.state == "pointingTowardsGoal":                    
                    w = Kpw*e_theta
                    v = 0.0

                               
                    if w >= 0.25:
                        w = 0.24
                    elif w <= -0.25:
                        w = -0.24               

                    mov.move(v,w)

                if mov.state == "travelingTowardsGoal":
                    w = Kpw*0.25*e_theta
                    #w = 0.0
                    #v = Kpv*e_pos
                    v = 0.07
                               
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
                    mov.state = "travelingTowardsGoal"
                        
                if ( abs(e_pos) <= 0.025 and mov.state == "travelingTowardsGoal"):                    
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
                    mov.reset()                    
                    mov.end_g2ps()                                                
            mov.rate.sleep()

        except rospy.ROSTimeMovedBackwardsException:
                rospy.logerr("ROS Time Backwards! reseting g2ps!")
                mov.reset() 
