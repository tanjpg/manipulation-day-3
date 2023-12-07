#!/usr/bin/env python
import os
import sys
import socket
import rclpy
import time

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from robotiq_msgs.msg import CModelStatus 
from robotiq_msgs.srv import CModelResponse 
from std_msgs.msg import Header

from robotiq_urcap_control.cmodel_urcap import RobotiqCModelURCap
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.time import Duration

class RobotiqUrcapPublisher(Node):
	def __init__(self):
		super().__init__('robotiq_publisher')
		self.pub = self.create_publisher(CModelStatus, 'gripper_publisher', 10)
		self.timer = self.create_timer(0.5, self.timer_callback)

	def timer_callback(self):
		global gripper
		status = gripper.getStatus()

		self.pub.publish(status)

		
class RobotiqUrcapService(Node):
	def __init__(self):
		super().__init__('robotiq_service')
		global gripper
		self.declare_parameter('gripper_service')
		self.declare_parameter('ip_address')
		self.declare_parameter('gripper_joint_publisher')
		self.declare_parameter('gripper_joint')
		self.declare_parameter('use_fake_hardware')
		# self.srv = self.create_service(CModelResponse, "/packing/gripper_service", self.response_callback)
		print('The current ur_address is: ' + self.get_parameter('ip_address').get_parameter_value().string_value)
		print('The gripper service: ' + self.get_parameter('gripper_service').get_parameter_value().string_value)

		gripper = RobotiqCModelURCap(self.get_parameter('ip_address').get_parameter_value().string_value)
		self.srv = self.create_service(CModelResponse, self.get_parameter('gripper_service').get_parameter_value().string_value, self.response_callback)
		self.joint_pub = self.create_publisher(JointTrajectory, self.get_parameter('gripper_joint_publisher').get_parameter_value().string_value, 10)


	def response_callback(self, request, response):
		global gripper
		if request.configuration == 1:
			print('End Effector is Robotiq 2f Gripper')
		if request.configuration == 2:
			print('End Effector is Robotiq Epick Gripper')
		if request.configuration != 1 and request.configuration != 2:
			response.response = 'You did not provide a suitable Configuration. Please input either 1(robotiq) or 2(epick)'
		
		# if request.configuration == 1:
		#   if request.robotiq2f_type != 85 and request.robotiq2f_type != 140:
		#     response.response = 'You did not provide a suitable robotiq2f_type. Please input either 85 or 140'
		#   elif request.rpr > 140:
		#     response.response = 'Object width is too wide for finger gripper'
		#   elif (request.rsp and request.rfr) > 255:
		#     response.response = 'rsp(Speed) or Force(rfr) is beyond gripper limits'
		#   else:
		#     gripper.sendCommand(request)
		else:
			if self.get_parameter('use_fake_hardware').get_parameter_value().bool_value:
				gripper.sendCommand(request)


		status = gripper.getStatus()

		joints_str = JointTrajectory()
		joints_str.header = Header()
		joints_str.header.stamp = self.get_clock().now().to_msg()
		joints_str.joint_names = [self.get_parameter('gripper_joint').get_parameter_value().string_value]
		point=JointTrajectoryPoint()
		point.positions = [1.0 - (request.rpr / 85.0)]
		point.time_from_start = Duration(seconds=0.1).to_msg()
		joints_str.points.append(point)
		self.joint_pub.publish(joints_str)
		
		## Check if object is detected by gripper
		if(status.gobj == 3):
			response.obj_detected = False
		elif(status.gobj == 2):
			response.obj_detected = True
		
		return response

def main(args=None):
	global gripper
	## Change according to IP address of UR. 
	# ur_address = "192.168.1.10"
	# print('The current ur_address is: ' + ur_address)
	rclpy.init(args=args)
	
	# gripper = RobotiqCModelURCap(ur_address)
	robotiq_service = RobotiqUrcapService()
	robotiq_publisher = RobotiqUrcapPublisher()

	executor = MultiThreadedExecutor(num_threads=4)
	executor.add_node(robotiq_publisher)
	executor.add_node(robotiq_service)
	try:
		 executor.spin()
	except KeyboardInterrupt:
		pass
	finally:
		executor.shutdown()
		robotiq_publisher.destroy_node()
		robotiq_service.destroy_node()

if __name__ == '__main__':
		main()
