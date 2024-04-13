#!/usr/bin/env python
# license removed for brevity
import rospy
import time
from std_msgs.msg import String
from geometry_msgs.msg import Point

PLANE_HEIGHT = 0.2 #Point.z

def location_init(location):
    location.x = None
    location.y = None
    location.z = None

def talker():
    strike_location = Point()
    location_init(strike_location)

    pub = rospy.Publisher('get_strike_locations', Point, queue_size=10)
    rospy.init_node('location_calculator', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():

        # if input("Enter Location (x, y, z) [press 'e' for exit]"):
        #     if input() == 'e':
        #         break 
        #     print("Enter location values now")
        #     for i in range(0, 2):
        #         if i == 0: strike_location.x = int(input())          
        #         if i == 1: strike_location.y = int(input())
        #         if i == 2: strike_location.z = int(input())
        strike_location.x = 0.2
        strike_location.y = 0.4
        strike_location.z = PLANE_HEIGHT
        rospy.loginfo(strike_location)
        pub.publish(strike_location)
        time.sleep(5)
        rate.sleep()        

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass