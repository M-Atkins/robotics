#!/usr/bin/env python
#Matthew Atkins
#University of Lincoln
#School of Computer Science
#ATK16657290

from cmath import isnan
import rospy
from geometry_msgs.msg import Twist
from math import radians
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from math import radians
from math import pi
from tf_conversions import transformations
import cv2
from cv2 import namedWindow, cvtColor, imshow
from cv_bridge import CvBridge
import numpy
from cv2 import waitKey
import rospy
import cv2
from threading import Thread, Lock
import math
import numpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg  import Image

import numpy as np

class MazeRunner:
    def __init__(self):
        self.rate = rospy.Rate(5)
        #initialise publishers and subscribers

        #For visual feed.
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        
        #Velocity 
        self.velPub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 1)
        self.odom = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laserscan_callback, queue_size = 1)

        self.laser = None
        self.angle = None
        self.isSearching = None
        self.stopAngle = None

        self.sBegin = False

        self.isMoving = None
        self.isTurningLeft = None
        self.isTurningRight = None
        #self.wallHit = None
        self.isStopped = None

        self.isSearching = True

        self.stopAngle = 0
        self.angle = 0
        self.check_left_and_right = False
        self.counter = 0
        self.rightTurnLoop = False
        self.wallHit = False
        self.space_in_front = 0
        self.initial_spin = False
        self.blue_seen_init_spin = False
        self.i = 0


    def image_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")# this decides output
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        #HSV colour ranges for each colour within the maze 
        lower_red = numpy.array([0, 50, 50])
        upper_red = numpy.array([20, 255, 255])
        lower_blue = numpy.array([80, 50, 50])
        upper_blue = numpy.array([130, 255, 255])
        lower_green = numpy.array([40,40,40])
        upper_green = numpy.array([70,255,255])
        #Masks
        red_mask = cv2.inRange(hsv,lower_red, upper_red)
        blue_mask = cv2.inRange(hsv,lower_blue, upper_blue)
        green_mask = cv2.inRange(hsv,lower_green, upper_green)

        h, w, c = cv_image.shape # height, width & color channels

        _, red_contours, _ = cv2.findContours(red_mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        _, green_contours, _ = cv2.findContours(green_mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        _, blue_contours, _ = cv2.findContours(blue_mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        #Search regions based on where that colour is found, then create mask for that region
        red_top = 9*h/10
        red_bottom = h 
        # red_left = 9*w/10
        # red_right = w
        red_mask[0:red_top, 0:w] = 0
        red_mask[red_bottom:h, 0:w] = 0

        #Get moments from the masks
        blue_moments = cv2.moments(blue_mask)
        red_moments = cv2.moments(red_mask)
        green_moments = cv2.moments(green_mask)

        #approach hint, then ignore
        if blue_moments['m00'] > 0:
            for c in blue_contours:
                area_of_color = cv2.contourArea(c) 
                if  area_of_color > 1.0 and self.space_in_front > 1.5 and self.check_left_and_right == False:#0.75:
                    self.blue_seen_init_spin = True
                    cx = int(blue_moments['m10']/blue_moments['m00'])
                    cy = int(blue_moments['m01']/blue_moments['m00'])
                    cv2.circle(cv_image, (cx, cy), 20, (120, 255, 255), -1)
                    err = cx - w/2
                    self.move_to_color(err)

        #when close enough, imagine red is a wall
        if red_moments['m00'] > 0  :
            for c in red_contours:
                area_of_color = cv2.contourArea(c) 
                if area_of_color > 1000.0:
                    self.wallHit = True

        #end main loop and approach, then stop when on the tile
        if green_moments['m00'] > 0:
            for c in green_contours:
                area_of_color = cv2.contourArea(c) 
                if area_of_color > 500.0 :
                    self.sBegin = False
                    cx = int(green_moments['m10']/green_moments['m00'])
                    cy = int(green_moments['m01']/green_moments['m00'])
                    cv2.circle(cv_image, (cx, cy), 20, (120, 255, 255), -1)
                    err = cx - w/2
                    self.move_to_color(err)
                    if self.space_in_front < 0.6:
                        print("DONE")
                        self.moveStop()

        cv2.imshow("window", cv_image)
        cv2.waitKey(3)

    def odom_callback(self, odom):
        self.angle = self.odom_orientation(odom.pose.pose.orientation)
        #print(self.angle)
    
    def odom_orientation(self, q):
        y, p, r = transformations.euler_from_quaternion([q.w, q.x, q.y, q.z])
        return y * 180 / pi
    
    def moveFront(self):
        self.isTurningLeft = False
        self.isTurningRight = False
        self.isStopped = False
        self.isMoving = True
        
        self.move(0, 0.2)
        rospy.loginfo("mFront")

    def turnLeft(self):
        self.isTurningLeft = True
        self.isTurningRight = False
        self.isStopped = False
        self.isMoving = False

        self.turn(45, 0)
        rospy.loginfo("mLeft")

    def turnRight(self):
        self.isTurningLeft = False
        self.isTurningRight = True
        self.isStopped = False
        self.isMoving = False

        self.turn(-45, 0)
        rospy.loginfo("mRight")

    def moveStop(self):        
        self.isTurningLeft = False
        self.isTurningRight = False
        self.isStopped = True
        self.isMoving = False 

        self.move(0, 0)
        rospy.loginfo("mStop")

    def move(self, ang, lin):
        t = Twist()
        t.linear.x = lin
        t.angular.z = radians(ang)
        self.velPub.publish(t)
        print("moving")

    def turn(self, ang, lin):
        t = Twist()
        t.linear.x = lin
        t.angular.z = radians(ang)
        self.velPub.publish(t)
        #rospy.sleep(1)
        print("turning")
        

    def checkFront(self, laser):
        print("IN front = " + str(min(laser.ranges[310:330])))
        if min(laser.ranges[310:330]) < 0.7:
            print("Object Detected!")
            return True
        else:
            return False

    def adjust(self, laser):
        laser_range = np.array(laser.ranges) #get laserscan ranges and convert to numpy arrray 
        left = laser_range[len(laser_range) - 10] # single hard left value 
        laser_range = laser_range[numpy.logical_not(numpy.isnan(laser_range))]

        right = laser_range[10] # single hard left value 
        err = right - left
        if math.isnan(err) == True:
            err = 0
        t = Twist()
        t.angular.z = -float(err) / 15
        t.linear.x = 0.2
        print("error\n", err)
        self.velPub.publish(t)

    def move_to_color(self, err):
        t = Twist()
        t.linear.x = 0.3
        t.angular.z = -float(err) / 50
        self.velPub.publish(t)

    def turn_x_degrees(self, radians):
        t = Twist()
        t.linear.x = 0
        t.angular.z = math.radians(radians)
        self.velPub.publish(t)

    def halfTurn(self, direction):
        if direction == "l":
            self.moveStop()
            for x in range(0,10):       
                self.turnLeft()
                self.rate.sleep()
            #rospy.sleep(1)

        if direction == "r":
            self.moveStop()
            for x in range(0,10):       
                self.turnRight()
                self.rate.sleep()
            #rospy.sleep(1)

    def laserscan_callback(self, laser):
        laser_range = np.array(laser.ranges) 
        middle_val = int(round(len(laser_range)/2)) 
        laser_range = laser_range[numpy.logical_not(numpy.isnan(laser_range))] 
        #front = min(laser_range[middle_val-7:middle_val+7])
        front = min(laser_range[middle_val-2:middle_val+2])
        self.space_in_front = min(laser_range[middle_val-2:middle_val+2])
        print(middle_val)
        print("callback - front = " + str(front))
        if self.initial_spin == False:
            print("initial spin")
            if self.blue_seen_init_spin == False or self.i <= 14:
                if self.i >= 14:
                    self.blue_seen_init_spin = True
                print(self.i)
                self.turn(-30, 0)
                self.i = self.i +1

                rospy.sleep(1)
            else:
                print("ending spin")
                self.inital_spin = True
                self.sBegin = True
        if self.sBegin == True:
            #
            print("main")
            self.moveFront()
            self.adjust(laser)
            self.wallHit = self.checkFront(laser)
            if self.wallHit == True:

                self.moveStop()
                self.check_left_and_right = True
                self.counter = 1
                self.sBegin = False

                #return
        if self.check_left_and_right == True:
            self.wallHit = False
            print("checking left and right - front = " + str(front))
            print(self.counter)
            if front > 1:
                self.check_left_and_right = False
                self.sBegin = True
                self.counter = 0
            #check left
            if self.counter == 1:
                print("check left")
                self.halfTurn("l")
                self.rate.sleep()

                rospy.sleep(1)
                self.counter += 1
            elif self.counter  == 2:
                #check right
                print("check right")
                self.rightTurnLoop = True
          
                ### RIGHT TURN LOOP ###
        if self.rightTurnLoop:
            print("correcting right")
            if front > 1.5 : # turn 10 degreees until these conditions are met
                self.sBegin = True
                self.rightTurnLoop = False
            else:

                for x in range(0,5):       
                    self.turn(-25, 0)
                    #self.rate.sleep()
                rospy.sleep(1)



    
rospy.init_node('mazerunner')
MazeRunner()
rospy.spin()
#cv2.destroyAllWindows()
