#!/usr/bin/env python

import roslib
import rospy
# Import the messages we're interested in sending and receiving
from geometry_msgs.msg import Twist  	 # for sending commands to the robot
from gazebo_msgs.srv import GetModelState    # for receiving Pose of simulated robots

COMMAND_PERIOD = 100 #ms

class BasicRobotController(object):
	def __init__(self):
 		
		# Allow the controller to publish to the /cmd_vel topic and thus control the robot
		self.pubCommand1 = rospy.Publisher('/robot1/cmd_vel',Twist)
                self.pubCommand2 = rospy.Publisher('/robot2/cmd_vel',Twist)
		self.pubCommand3 = rospy.Publisher('/robot3/cmd_vel',Twist)
                self.pubCommand4 = rospy.Publisher('/robot4/cmd_vel',Twist)
		# Setup regular publishing of control packets
		self.command1 = Twist()
		self.commandTimer1 = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0),self.SendCommand1)
		self.command2 = Twist()
		self.commandTimer2 = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0),self.SendCommand2)
		self.command3 = Twist()
		self.commandTimer3 = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0),self.SendCommand3)
		self.command4 = Twist()
		self.commandTimer4 = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0),self.SendCommand4)

         
        def robot1(self):
                try:
                    subPose = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
                    robot1_Pose=subPose("iRobot_create1", "link")
                    return robot1_Pose.pose.position.x, robot1_Pose.pose.position.y, robot1_Pose.pose.position.z, robot1_Pose.pose.orientation.x, robot1_Pose.pose.orientation.y, robot1_Pose.pose.orientation.z, robot1_Pose.pose.orientation.w
                except rospy.ServiceException as e:
                    rospy.loginfo("Get Model State service call failed:  {0}".format(e))

        def robot2(self):
                try:
                    subPose = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
                    robot2_Pose=subPose("iRobot_create2", "link")
                    return robot2_Pose.pose.position.x, robot2_Pose.pose.position.y, robot2_Pose.pose.position.z, robot2_Pose.pose.orientation.x, robot2_Pose.pose.orientation.y, robot2_Pose.pose.orientation.z, robot2_Pose.pose.orientation.w
                except rospy.ServiceException as e:
                    rospy.loginfo("Get Model State service call failed:  {0}".format(e))

        def robot3(self):
                try:
                    subPose = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
                    robot3_Pose=subPose("iRobot_create3", "link")
                    return robot3_Pose.pose.position.x, robot3_Pose.pose.position.y, robot3_Pose.pose.position.z, robot3_Pose.pose.orientation.x, robot3_Pose.pose.orientation.y, robot3_Pose.pose.orientation.z, robot3_Pose.pose.orientation.w
                except rospy.ServiceException as e:
                    rospy.loginfo("Get Model State service call failed:  {0}".format(e))

        def robot4(self):
                try:
                    subPose = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
                    robot4_Pose=subPose("iRobot_create4", "link")
                    return robot4_Pose.pose.position.x, robot4_Pose.pose.position.y, robot4_Pose.pose.position.z, robot4_Pose.pose.orientation.x, robot4_Pose.pose.orientation.y, robot4_Pose.pose.orientation.z, robot4_Pose.pose.orientation.w
                except rospy.ServiceException as e:
                    rospy.loginfo("Get Model State service call failed:  {0}".format(e))
                

	def SetCommand1(self,pitch=0,yaw_velocity=0):
		# Called by the main program to set the current command
		self.command1.linear.x  = pitch
		self.command1.angular.z = yaw_velocity

	def SetCommand2(self,pitch=0,yaw_velocity=0):
		# Called by the main program to set the current command
		self.command2.linear.x  = pitch
		self.command2.angular.z = yaw_velocity

	def SetCommand3(self,pitch=0,yaw_velocity=0):
		# Called by the main program to set the current command
		self.command3.linear.x  = pitch
		self.command3.angular.z = yaw_velocity

	def SetCommand4(self,pitch=0,yaw_velocity=0):
		# Called by the main program to set the current command
		self.command4.linear.x  = pitch
		self.command4.angular.z = yaw_velocity

	def SendCommand1(self,event):
	        self.pubCommand1.publish(self.command1)

	def SendCommand2(self,event):
	        self.pubCommand2.publish(self.command2)

	def SendCommand3(self,event):
	        self.pubCommand3.publish(self.command3)

	def SendCommand4(self,event):
	        self.pubCommand4.publish(self.command4)

