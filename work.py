#by Jonathan Ryan RYA15614996
#reference:
#Morgan Quigley, Brian Gerkey, William D. Smart.(2015)Programming Robots with ROS: A Practical Introduction to the Robot Operating System.O'Reilly Media
import rospy
import math
import numpy
import cv2
import cv_bridge
from math import radians
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image

class MazeRunner():


    def __init__(self):
        
        #intialised at 1 to satisfy first movement conditions as sensors update after movement
        self.range_ahead = 1.0
        self.range_right = 1.0
        self.range_left = 1.0
        self.stop = False
        self.masks = [0,0]#contains the moment contor value from the moments of the masks
        
        rospy.loginfo("Starting sensory node")
        self.sub = rospy.Subscriber('scan', LaserScan, callback=self.collision_detection)
        
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image,
                                          self.image_callback)

        rospy.loginfo("Starting MazeRunner node")
        self.pub = rospy.Publisher("/mobile_base/commands/velocity", Twist)
        

    #"laser sensor" is just the camera
    #msg.ranges is an array equal in size to the width of view of the camera, each element is distance value from the robot
    #information on laser scanner found: page(104 of cited book)
    def collision_detection(self,msg):
        self.range_ahead = msg.ranges[len(msg.ranges)/2]
        self.range_right = msg.ranges[0]
        self.range_left = msg.ranges[len(msg.ranges)-1]
        
        print "range ahead: %0.1f" % self.range_ahead, "left: %0.1f" % self.range_left, "right: %0.1f" % self.range_right
        
    #function adapted from: page 207 of cited book
    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #use bgr values for colour detection 
        lower = numpy.array([0,0,0])
        green = numpy.array([10, 255, 10])
        red = numpy.array([10,10,255])
        

        mask_green = cv2.inRange(image, lower, green)
        mask_red = cv2.inRange(image, lower, red)
        h, w, d = image.shape
        
        #boundry selection (this is all to get the image just infront the robot)
        search_top = 3*h/4 #top quaters of the screen
        search_bottom = search_top + 20 #should leave a 20 pixel window from the bottom

        mask_green[0:search_top, 0:w] = 0
        mask_green[search_bottom:h, 0:w] = 0

        mask_red[0:search_top, 0:w] = 0
        mask_red[search_bottom:h, 0:w] = 0


        #'m00' referes to moment contor 
        #contor is the boundry of pixels with the same intensity, objects in the mask will appear with 
        #the same intensity (white)
        #holds the pixel count value of the contor for the forward() function thresholds
        self.masks = [cv2.moments(mask_green)['m00'],cv2.moments(mask_red)['m00']]
        
        
   


    def forward(self):
        r = rospy.Rate(5)
        rospy.loginfo("moving forward 0.2m/s")
        move_cmd = Twist()
        
        while (self.stop != True):
            #threshold found by placing robot infront of coloured square
            #red detection        
            while(self.masks[1] > 2000000): 
                
                print "red found turning around", self.masks[1]
                for x in range (20):
                    move_cmd.linear.x = 0.0
                    move_cmd.angular.z = radians(45)#radians part taken from squares example in the github
                    self.pub.publish(move_cmd)
                    r.sleep()

            #green detection
            while(self.masks[0] > 2000000):
               
                print "green detected: ",self.masks[0]
                for x in range (15):
                    move_cmd.linear.x = 0.2
                    move_cmd.angular.z = 0.0
                    self.pub.publish(move_cmd)
                    r.sleep()
                #programing finished
                self.stop = True

            #counter for how many times corner nav has been triggered
            flip = 0
            #move forward 
            if(self.range_ahead >= 1):
                move_cmd.linear.x = 0.2
                move_cmd.angular.z = 0
            else:#while theres not enough space to move forward try to turn
                while(self.range_ahead <= 1):
                    print "flip count: ", flip
                    #print flip
                    if(flip > 12):#deadend dection (most corners can be done in under 12 turns so above is usually a deadend)
                        for x in range (20):
                            move_cmd.linear.x = 0.0
                            move_cmd.angular.z = radians(45)
                            self.pub.publish(move_cmd)
                            r.sleep()
                    
                    #navigate corners (move towards more space)
                    if(self.range_left >= self.range_right):
                        move_cmd.linear.x = 0.0
                        move_cmd.angular.z = radians(45)
                        flip = flip + 1 
                    else:#turn right
                        move_cmd.linear.x = 0.0
                        move_cmd.angular.z = -radians(45)
                        flip = flip + 1 

                    self.pub.publish(move_cmd)
                    r.sleep()

            #avoids bumps
            if (self.range_left <= 0.7):
                move_cmd.linear.x = 0.2
                move_cmd.angular.z = -0.4#turn right

            if (self.range_right <= 0.7):
                move_cmd.linear.x = 0.2
                move_cmd.angular.z = 0.4#turn left
            
            self.pub.publish(move_cmd)
            r.sleep()
            


if __name__ == '__main__':
    rospy.init_node("MazeRunner")
    mr = MazeRunner()
    mr.forward()
    


