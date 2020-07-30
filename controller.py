#!/usr/bin/env python


import rospy  #Importing the necessary packages
import rosbag
import time
from statistics import mean
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

#A class to define publisher and Subscriber
class pubsub():
    
    def __init__(self):

        #subscribing into laserdata
        self.laser_scan = rospy.Subscriber('/base_scan',LaserScan,self.scan)
        
        #publishing the velocity
        self.mov= rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        self.speed = Twist()
        

    #laserscanning and finding the wall
    def scan(self,laserdata):
        global ctrl
        self.w = False
        self.vx = self.vy = self.vw= 0.0
        scan_data = list(laserdata.ranges)
        
        if not ctrl:
            self.speed.linear.x = 0.5
            self.speed.linear.y = 0.0
            self.speed.angular.z= 0.0
            self.mov.publish(self.speed)
            if min(scan_data[70:80]) < 0.4 :
                self.speed.linear.y = 0.0
                self.speed.linear.x = 0.0
                self.speed.angular.z = 0.0
                self.mov.publish(self.speed)
                self.w = True
            else:
                pass
        else:
            self.w = True

        if self.w:
            
            if (min(scan_data[70:80]) < 0.4) and (min(scan_data[147:148]) < 0.7) : #object in the front turn right
                self.vx = 0.0
                self.vy = 0.0
                self.vw = -0.4
                time.sleep(0.3)

            elif (min(scan_data[100:120]) < 0.3) : #Robot near to left wall
                self.vx = 0.0
                self.vy = 0.0
                self.vw = -0.3  
                
            elif min(scan_data[70:80])< 0.3: #robot fvery near to the front
                self.vx = - 0.2
                self.vy = 0.0  
                self.vw = 0.0
                
            elif min(scan_data[147:148])> 0.7: #robot far away from the reference wall
                self.vx = 0.0
                self.vy = 0.2
                self.vw = 0.0
                
            elif min(scan_data[147:148])< 0.4: #robot very near to the reference wall
                self.vx = 0.0
                self.vy = 0.0
                self.vw = -0.2

            elif (min(scan_data[20:40]) < 0.3): #robot near to right wall
                self.vx = 0.0
                self.vy = 0.0
                self.vw = 0.3
                
            else:
                self.vx = 0.3
                self.vy = 0.0
                self.vw = 0.0
                   
            self.control()
        
        
    def control(self):
        global ctrl
        ctrl = True
        #Publishing the velocities 
        if self.w and ctrl:
            self.speed.linear.x = self.vx
            self.speed.linear.y = self.vy
            self.speed.angular.z = self.vw
            self.mov.publish(self.speed)


     

       

   
                
            

        


    








if __name__ == '__main__':
    ctrl = False
    rospy.init_node('youbot')
    pubsub()
    rospy.spin()
    
                



     

    
