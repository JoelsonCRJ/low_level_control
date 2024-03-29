#!/usr/bin/python
import rospy
import smbus # I2C na Rasp
from geometry_msgs import Twist #message type given from /odom topic
import numpy as np
# MD22
# Assuming I2C Bus address 0xB0 ->  mode switches: on on on on


class LowLevelControl(object):
	def __init__(self):
        self.rospy = rospy
        self.rospy.init_node("Low_Level_Controller", anonymous = True)
        self.rospy.loginfo("Starting Low Level Controller Node")

        #try:
        self.init_i2c()
        #except:
            # Throw Error
            # Close node

        self.initParameters()
        self.initSubscribers()
        self.initPublishers()
        self.initVariables()

		self.mainController()

    def init_i2c(self):
        # Change!!!
        # Needs testing
        #self.bus = smbus.SMBus(1)    # 0 = /dev/i2c-0 (port I2C0), 1 = /dev/i2c-1 (port I2C1)
        #DEVICE_ADDRESS = 0x58      #will be left shifted to add the read write bit - becomes 0xB0
        #DEVICE_REG_MODE1 = 0x00
        return

    def initParameters(self):
	    self.control_rate = self.rospy.get_param("~control_rate", 100)
        self.velTopic = self.rospy.get_param("~vel_topic","/cmd_vel")

        return

    def initSubscribers(self):
        # Subscribe odometry/pose (encoder+imu) and desired velocity (high level control output)
        self.subPose = self.rospy.Subscriber("/odom", Twist, self.callback_pose)
        self.
 		return
    
    def initPublishers(self):
        # Publish driver signal (logging purpose)
        self.pubVel = self.rospy.Publisher(self.velTopic, Twist, queue_size = 10)
        return

    def initVariables(self):
        self.rate = self.rospy.Rate(self.control_rate)
        self.wheels_radio_size=0 #define the wheels radio size
        self.whels_distance =0 #define the axis length
        #init vel control (reference) -> Driver's command ?
        #init vel encoders (pose)     -> Encoder's reading before IMU fusion ?
        self.vel_encoders=np.zeros(2,dtype=float) #pose
        self.linear_velocity=0
        self.angular_velocity=0
        return


    def pubVel(self):
        self.pubVel.publish(self.speeds)
        return
        
	def callback_high_level_control(self,msg):
        
		return

	def callback_pose(self,msg):
        #por exemplo self.pose_last = msg
        self.speeds = msg #twist type
		return

    def vel_walker_to_wheels(self,linear_vel,angular_vel):
        # from lin (m/s) and ang (rad/s) to vel_l and vel_r (m/s)
        right_wheel_vel = linear_vel + ((angular_vel*self.whels_distance)/2)
        left_wheel_vel = linear_vel - ((angular_vel*self.whels_distance)/2)
        return left_wheel_vel, right_wheel_vel

    def pid_controller(self, vel_ref_wheel, vel_wheel):
        # get error
        # pid
        return #control_signal

    def construct_i2c_msg(self,value_1,value_2):
        return #msg

    def send_i2c(self,msg):
        #self.bus.write_byte_data(DEVICE_ADDRESS, DEVICE_REG_MODE1, 0x80)
        return

	def main_controller(self):
		while not self.rospy.is_shutdown():
            # update vel_ref
            # get vel_ref for each wheel
            # update vel_walker
            # get vel_walker for each wheel
            # get call PID for left wheel
            # get call PID for right wheel
            # construct i2c message
            # send i2c
            
            # keep log (1 hz) 
			self.rate.sleep()


if __name__ == '__main__':
	print("Starting Low Level Controller")
	try:
		llc = LowLevelControl()
	except rospy.ROSInterruptException:
		pass
print('Exiting Low Level Controller')
