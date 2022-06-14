#!/usr/bin/env python
# coding=utf-8
import sys
import numpy as np
import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, EmptyRequest
#from roseus.msg import StringStamped

#Creamos la clase
class iois_checker():
    def __init__(self):
        #Inicializamos el nodo
        rospy.init_node("instructAndInterruptChecker")
        #Creamos el publisher
        self.pub_curr_ioi = rospy.Publisher("/curr_instruction_or_interrupt", String, queue_size=1)
        
        #Creamos los subscribers
        self.sub_crosswalk_det = rospy.Subscriber("/crosswalk_in_scene",Bool, self.on_crosswalk_in_scene_callback)
        self.sub_curr_ioi = rospy.Subscriber("/last_instructions_and_interrupts",String, self.on_last_iois_callback)
        self.sub_line_follower = rospy.Subscriber("/curr_traffic_lights",String, self.on_curr_traffic_light)
        self.sub_g2ps = rospy.Subscriber("/curr_traffic_signs",String,self.on_curr_traffic_signs)

        #self.last_ioi_time = None
        self.curr_ioi = "none"        
        self.curr_traffic_lights = []
        self.curr_traffic_signs = []
        self.curr_iois = []
        self.last_iois = []
        self.crosswalk_in_scene = False
        
        self.curr_ioi_msg = String()
        
        self.speedInterruptions = ["red traffic light", "yellow traffic light", "green traffic light", "stop sign", "end of prohibition sign"]
        self.instructors = ["straight sign", "turn right ahead sign"]        
        self.prorize_result = []
        self.compare_result = []

        #Declaramos que vamos a mandar 20 mensajes por segundo.
        self.rate = rospy.Rate(30)
        self.rateInt = 30                          
                    
        rospy.on_shutdown(self.end_callback)

    def on_last_iois_callback(self, data):
        self.last_iois = eval(data.data)

    def on_crosswalk_in_scene_callback(self, data):
        self.crosswalk_in_scene = data.data        

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
        if  ("red traffic light" in iois or "yellow traffic light" in iois or "green traffic light" in iois) and (self.crosswalk_in_scene):
            if "red traffic light" in iois:                 
                self.prorize_result.append("red traffic light")
            elif "yellow traffic light" in iois:
                self.prorize_result.append("yellow traffic light")
            elif "green traffic light" in iois:
                self.prorize_result.append("green traffic light")
        if "stop sign" in iois:
            self.prorize_result.append("stop sign")
        if "end of prohibition sign" in iois:
            self.prorize_result.append("end of prohibition sign")        
        return self.prorize_result
        
    def compare_curr_iois_with_last_iois(self,curr, last):
        #self.compare_result = []
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
