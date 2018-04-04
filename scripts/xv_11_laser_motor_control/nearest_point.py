#! /usr/bin/python

# MIT License
#
# Copyright (c) 2018 Julien BAYLE
#  
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import roslib
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import math

"""
Publish LIDAR closest object distance and angle
"""
class ScanToNearestPoint:
    def __init__(self):
        self.pubAngle = rospy.Publisher('scan_min_distance_angle', Float64, queue_size=10)
        self.pubDistance = rospy.Publisher('scan_min_distance', Float64, queue_size=10)
        self.sub = rospy.Subscriber('scan', LaserScan, self.update)

    def update(self, msg):
	angle = msg.angle_min
        d_angle = msg.angle_increment
        min_distance_angle = angle
        min_distance = msg.range_max

        for r in msg.ranges:
            if r > 0.2 and r < min_distance:
                min_distance = r
                min_distance_angle = angle
            angle += d_angle
        
        self.pubAngle.publish(min_distance_angle*180/math.pi)
        self.pubDistance.publish(min_distance*100)
        #rospy.loginfo("Distance %f Angle %f", min_distance*100, min_distance_angle*180/math.pi)

if __name__ == '__main__':
    rospy.init_node('scan_to_min_distance_with_angle')
    s = ScanToNearestPoint()
    rospy.spin()
