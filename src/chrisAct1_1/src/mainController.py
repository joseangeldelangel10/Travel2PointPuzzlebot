#!/usr/bin/env python
# coding=utf-8
import sys
import numpy as np
import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
#from roseus.msg import StringStamped

#Creamos la clase
class mainController():
    def __init__(self):
        #Inicializamos el nodo
        rospy.init_node("mainController")
        #Creamos el publisher
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pub_curr_action = rospy.Publisher("/curr_action", String, queue_size=1)
        self.pub_g2p_mode = rospy.Publisher("/g2p_mode", String, queue_size=1)
        self.pub_last_ioi = rospy.Publisher("/last_instructions_and_interrupts", String, queue_size=1)        
        
        #Creamos los subscribers
        self.sub_curr_ioi = rospy.Subscriber("/curr_instruction_or_interrupts",String, self.on_curr_ioi_callback)
        self.sub_line_follower = rospy.Subscriber("/cmd_vel_LF",Twist, self.on_line_follower_vel_callback)
        self.sub_g2ps = rospy.Subscriber("/cmd_vel_g2p",Twist,self.on_go_to_points_vel_callback)
        #self.sub_g2ps_success = rospy.Subscriber("/g2p_success",Bool,self.on_g2p_success_msg)

        s = rospy.Service('end_g2ps', Empty , self.end_g2ps_callback)

        #self.last_ioi_time = None
        self.last_ioi_time = 0
        
        self.curr_ioi_time = None
        self.curr_ioi_data = None
        self.line_follower_v = 0.0
        self.line_follower_w = 0.0        
        self.go_to_points_v = 0.0
        self.go_to_points_w = 0.0
        #self.g2ps_succeded = False
        self.odometry_is_reseted = False
        self.vel_mult = 1.0
        self.last_iois = []
        
        self.cmd_vel_msg = Twist()
        self.curr_action_msg = String()
        self.g2p_mode_msg = String()
        self.last_iois_msg = String()

        self.speedInterruptions = ["red traffic light", "yellow traffic light", "green traffic light", "stop sign", "end of prohibition sign"]
        self.instructors = ["straight sign", "turn right ahead sign"]
        self.speedInterruptionsCoefficients = {"red traffic light":0.0, "yellow traffic light":0.5, "green traffic light":1.0, "stop sign":0.0, "end of prohibition sign":1.25}
        self.action_stack = ["line follower"]        

        #Declaramos que vamos a mandar 20 mensajes por segundo.
        self.rate = rospy.Rate(30)
        self.rateInt = 30

        rospy.wait_for_service("reset_odometry")
        try:
            self.reset_odometry = rospy.ServiceProxy("reset_odometry", Empty)
        except rospy.ServiceException as e:
            print(e)                  
    
        #Creamos un función de que hacer cuando haya un shutdown        
        rospy.on_shutdown(self.end_callback)

    def end_g2ps_callback(self, req):
        self.odometry_is_reseted = False
        self.pop_from_action_stack()
        return EmptyResponse() 

    def on_curr_ioi_callback(self, data):
        self.curr_ioi_time = self.last_ioi_time + 1
        self.curr_ioi_data = data.data

    def on_line_follower_vel_callback(self, data):
        self.line_follower_v = data.linear.x
        self.line_follower_w = data.angular.z

    def on_go_to_points_vel_callback(self, data):
        self.go_to_points_v = data.linear.x
        self.go_to_points_w = data.angular.z

    def end_callback(self):
        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_msg.angular.z = 0.0        
        self.pub_cmd_vel.publish(self.cmd_vel_msg)

    def on_g2p_success_msg(self, data):
        self.g2ps_succeded = data.data 

    def addInstruction(self, string):
        if string not in self.action_stack:
            print("sring was not in action stack")
            if string == self.instructors[0]:
                self.action_stack += ["go straight"]
            elif string == self.instructors[1]:
                self.action_stack += ["turn right ahead"]

    def publish_vel(self, v, w):
        self.cmd_vel_msg.linear.x = v
        self.cmd_vel_msg.angular.z = w
        self.pub_cmd_vel.publish(self.cmd_vel_msg)

    def publish_current_action(self, action):
        self.curr_action_msg.data = action
        self.pub_curr_action.publish(self.curr_action_msg)

    def publish_g2p_mode(self, mode):
        self.g2p_mode_msg.data = mode
        self.pub_g2p_mode.publish(self.g2p_mode_msg)

    def pop_from_action_stack(self):
        if len(self.action_stack) > 1:
            self.action_stack = self.action_stack[:-1]

    def main(self):
        while not rospy.is_shutdown():
            #TODO PUBLISH LAS IOIS MESSAGE

            # WE CHECK FOR INTERRUPTS AND INSTRUCTORS (iois)
            if self.curr_ioi_data != None:
                if self.last_ioi_time != None:
                    if self.last_ioi_time < self.curr_ioi_time:                        
                        if self.curr_ioi_data in self.instructors:
                            self.addInstruction(self.curr_ioi_data)
                            print("instructor detected")                                                
                        elif self.curr_ioi_data in self.speedInterruptions:
                            print("speedInterruption detected")
                            self.vel_mult = self.speedInterruptionsCoefficients[self.curr_ioi_data]
                else:
                    #is the fist iteration
                    if self.curr_ioi_data in self.speedInterruptions:
                        self.vel_mult = self.speedInterruptionsCoefficients[self.curr_ioi_data]
                    elif self.curr_ioi_data in self.instructors:
                        self.addInstruction(self.curr_ioi_data)
                self.last_iois.append(self.curr_ioi_data)
                self.last_ioi_time = self.curr_ioi_time 

            if self.action_stack[-1] == "line follower":                
                self.publish_current_action("line follower")
                self.publish_vel(self.line_follower_v*self.vel_mult, self.line_follower_w*self.vel_mult)
            elif self.action_stack[-1] == "go straight":
                # this state ends when g2ps node calls the end_g2ps service                
                if not self.odometry_is_reseted:
                    self.reset_odometry()
                    self.odometry_is_reseted = True
                self.publish_current_action("go to point")
                self.publish_g2p_mode("go straight")
                self.publish_vel(self.go_to_points_v*self.vel_mult, self.go_to_points_w*self.vel_mult)
            elif self.action_stack[-1] == "turn right ahead":                
                # this state ends when g2ps node calls the end_g2ps service
                if not self.odometry_is_reseted:
                    self.reset_odometry()
                    self.odometry_is_reseted = True
                self.publish_current_action("go to point")
                self.publish_g2p_mode("turn right ahead")
                self.publish_vel(self.go_to_points_v*self.vel_mult, self.go_to_points_w*self.vel_mult)

            #print("action_stack is:" + str(self.action_stack))
            self.rate.sleep()

#Si el archivo es corrido directametne y no llamado desde otro archivo corremos
if __name__ == "__main__":
    controller = mainController()
    controller.main()

            
    #mov.main()
