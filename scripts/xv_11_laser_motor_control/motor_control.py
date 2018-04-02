#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16
import RPi.GPIO as GPIO

class xv11MotorControl:
    
    def __init__(self, initialPWM):
        """Init Raspberry PWM output"""
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(12, GPIO.OUT, initial = GPIO.LOW)
        self.motorLidar = GPIO.PWM(12, 200)
    
        # Define an initial speed as the lidar driver
        # only publish motor speed in RPM if motor runs
        self.motorLidar.start(initialPWM)
        self.currentPWM = initialPWM
    
    def updatePWM(self, newPWM):
        """Change PWM value"""
        self.currentPWM = max(min(newPWM, 100), 0)
        self.motorLidar.ChangeDutyCycle(self.currentPWM)
        self.pwmPublisher.publish(self.currentPWM)
        #rospy.loginfo("Lidar motor PWM modified to %d", self.currentPWM)

    def adaptPWM(self, data):
        """Adapt PMW to control lidar speed"""
        rpm = float(data.data)
        #rospy.loginfo("Lidar speed %d RPM, actual PWM %d", rpm, self.currentPWM)
        if rpm > 600:
            rospy.loginfo("Invalid Lidar speed, nothing to do")
        else:
            error = 300 - rpm
            self.updatePWM(self.currentPWM + (error / 5))

    def run(self):
        def callback(data):
            return self.adaptPWM(data)
        try:
            rospy.init_node("lidar_motor_control")
            self.pwmPublisher = rospy.Publisher("laser_pwm", UInt16, queue_size=10)
            rospy.Subscriber("rpms", UInt16, callback)

            # Loop until this node is stopped
            rospy.spin()
    
        except KeyboardInterrupt:  
            pass
    
        finally:
            self.motorLidar.stop()
            rospy.loginfo("Lidar motor speed controler has been stopped")


if __name__ == '__main__':
    initialPWM = rospy.get_param("init_pwm", 50)
    controller = xv11MotorControl(initialPWM)
    controller.run()
