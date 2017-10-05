#!/usr/bin/env python
import unittest
import rospy
import threading
import serial
import time
from std_msgs.msg import Header, String
from mil_tools import thread_lock
from ros_alarms import AlarmBroadcaster, AlarmListener
from navigator_kill_board import constants
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from navigator_kill_board import SimulatedKillBoard
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from ros_alarms.msg import Alarm as AlarmMsg

class killtest(unittest.TestCase):
	
	def __init__(self,*args):	
		self._current_alarm_state=None		
		rospy.Subscriber("/diagnostics", DiagnosticArray, self.callback)
		super(killtest, self).__init__(*args)
		self.AlarmListener=AlarmListener('hw-kill')
		self.AlarmBroadcaster=AlarmBroadcaster('kill')
		self.AlarmBroadcaster.clear_alarm() 
		
	def callback(self,msg):
	    	self._current_alarm_state = msg
					
	def test_AA_initial_state(self):                 
		self.AlarmBroadcaster.clear_alarm()
		rospy.sleep(0.05)					
		self.assertEqual(self.AlarmListener.is_cleared(), True, msg='current state of kill signal is raised')
		
	def test_AB_computer(self):
		self.assertEqual(self.AlarmListener.is_cleared(), True, msg='current state of kill signal is raised')
		
		self.AlarmBroadcaster.raise_alarm()
		rospy.sleep(0.05)	
		self.assertEqual(self.AlarmListener.is_raised(), True, msg='COMPUTER raise not worked')

		self.AlarmBroadcaster.clear_alarm()      
		rospy.sleep(0.05)
		self.assertEqual(self.AlarmListener.is_cleared(), True, msg='COMPUTER shutdown not worked') 
			

	def test_AC_button_front_port(self):
		self.assertEqual(self.AlarmListener.is_cleared(), True, msg='current state of kill signal is raised')
		
		rospy.wait_for_service('/kill_board_driver/BUTTON_FRONT_PORT')
		try:
			bfp=rospy.ServiceProxy('/kill_board_driver/BUTTON_FRONT_PORT',SetBool)
			
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

						
		bfp(True)
		rospy.sleep(0.05)		
		self.assertEqual(self.AlarmListener.is_raised(), True, msg='BUTTON_FRONT_PORT raise not worked') 
		
		
		bfp(False)
		self.AlarmBroadcaster.clear_alarm()
		rospy.sleep(0.05)
		self.assertEqual(self.AlarmListener.is_cleared(), True, msg='BUTTON_FRONT_PORT shutdown not worked. State = {}'.format(self._current_alarm_state))
		
		
		

	def test_AD_button_aft_port(self):
		self.assertEqual(self.AlarmListener.is_cleared(), True, msg='current state of kill signal is raised')
		rospy.wait_for_service('/kill_board_driver/BUTTON_AFT_PORT')
		try:
			bap=rospy.ServiceProxy('/kill_board_driver/BUTTON_AFT_PORT',SetBool)
			
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		
		bap(True)
		rospy.sleep(0.05)
		self.assertEqual(self.AlarmListener.is_raised(), True, msg='BUTTTON_AFT_PORT raise not worked')
		
		bap(False)
		self.AlarmBroadcaster.clear_alarm()
		rospy.sleep(0.05)
		self.assertEqual(self.AlarmListener.is_cleared(), True, msg='BUTTTON_AFT_PORT shutdown not worked. State = {}'.format(self._current_alarm_state))

		
	
	def test_AE_button_front_starboard(self):
		self.assertEqual(self.AlarmListener.is_cleared(), True, msg='current state of kill signal is raised')
		rospy.wait_for_service('/kill_board_driver/BUTTON_FRONT_STARBOARD')
		try:
			bfs=rospy.ServiceProxy('/kill_board_driver/BUTTON_FRONT_STARBOARD',SetBool)
			
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		
		bfs(True)
		rospy.sleep(0.05)
		self.assertEqual(self.AlarmListener.is_raised(), True, msg='BUTTON_FRONT_STARBOARD raise not worked')
		
		bfs(False)
		self.AlarmBroadcaster.clear_alarm()	
		rospy.sleep(0.05)
		self.assertEqual(self.AlarmListener.is_cleared(), True, msg='BUTTON_FRONT_STARBOARD shutdown not worked. State = {}'.format(self._current_alarm_state))
		
		
	def test_AF_button_aft_starboard(self):
		self.assertEqual(self.AlarmListener.is_cleared(), True, msg='current state of kill signal is raised')		
		rospy.wait_for_service('/kill_board_driver/BUTTON_AFT_STARBOARD')
		try:
			bas=rospy.ServiceProxy('/kill_board_driver/BUTTON_AFT_STARBOARD',SetBool)
			
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

		bas(True)
		rospy.sleep(0.05)
		self.assertEqual(self.AlarmListener.is_raised(), True, msg='BUTTON_AFT_STARBOARD raise not worked')
		
		bas(False)
		self.AlarmBroadcaster.clear_alarm()
		rospy.sleep(0.05)
		self.assertEqual(self.AlarmListener.is_cleared(), True, msg='BUTTON_AFT_STARBOARD shutdown not worked. State = {}'.format(self._current_alarm_state))
		
		
		

	
	def test_AG_remote(self):
		self.assertEqual(self.AlarmListener.is_cleared(), True, msg='current state of kill signal is raised')
		pub = rospy.Publisher('/network', Header, queue_size=10)
		rate = rospy.Rate(10)
		t_end=rospy.Time.now()+rospy.Duration(3)	        
		while rospy.Time.now()<t_end:
			hello_header = Header()
			hello_header.stamp = rospy.Time.now()
			rospy.loginfo(hello_header)
			pub.publish(hello_header)
			rate.sleep()
		
		self.assertEqual(self.AlarmListener.is_cleared(), True, msg='REMOTE shutdown not worked. State = {}'.format(self._current_alarm_state))
		
		rospy.sleep(8.05)
		
		self.assertEqual(self.AlarmListener.is_raised(), True, msg='REMOTE raised not worked. State = {}'.format(self._current_alarm_state))

#1. Nothing wrong was found when running the dirver
#2. Setting buttons wrong not working, so is remote
#3. is_clear(), can this be false when overall is false


if __name__=="__main__":
	rospy.init_node('lll',anonymous=True)
	import rostest
    	rostest.rosrun('navigator_kill_board', 'kill_board_test', killtest)
	unittest.main()

# P1. Call the service, then set the value of each button True, how to set arguments to this service? 
# ANS: Name=rospy.ServiceProxy(,), name(True or False).
# P2. Am I using the right name? 
# ANS: AlarmListener.is_raised() is the overall status showing if the computer recieved the request.
# P3. Call the service, same as in terminal, something wrong with name SetBool?
# ANS: From std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse. Why I have problem before about SetBool is I didnt import this class.
# P4. How to shut the computer kill request?
# ANS: AlarmBroadcaster.clear_alarm(), if I want to raise for computer, use AlarmBroadcaster.raise_alaem()
# P5. How to give arguent to remote?
# ANS: Creating a ros pubisher named '/network', sending message to subscriber and then stop
# P6. How to show the diagnostics correctly?
# ANS: In the script kill_board_driver, I can see all the changes and updates in the command, so maybe I dont need to show the diagonostics. If I really want, use subscriber for diagonostic and callback, then print it out
# P7. What if the scipt just doesn't work?
# ANS: Print the diagnostics everytime, thus I know which part is going wrong and know the information of diagnostics
