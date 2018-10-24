#!/usr/bin/env python
from __future__ import division
import txros
import numpy as np
import mil_tools
from mil_misc_tools.text_effects import fprint
from navigator import Navigator
import math
from twisted.internet import defer
from mil_tools import rosmsg_to_numpy

___author___ = "Marshall Rawson"
#This mission takes care of the second part of the qualifier objective
class StartGateMarshall(Navigator):
	#this method gets the location of the nearest black totem and the scan the code platform
	#from the PCODAR database
	
	#runs the scan the code preception script eventually
	def get_scan_the_code(self):
		#currently hard coded, no STC that works yet :/
		return False

	#returns the xy of target totem and unit vector from target to non target totem
	@txros.util.cancellableInlineCallbacks
	def get_bouy_go_round_target(self):
		return_array = []
		
		#gets the xy and state of the scan the code from the database
		scan_the_code = np.array([])
		res = yield self.database_query('stc_platform')
		#makes sure that only 1 scan the code exists
		assert len(res.objects) == 1
		#raises error if the scan the code platform is nto
		if not res.found:
			raise TaskException(query + ' not found in object database')
		point = rosmsg_to_numpy(res.objects[0].pose.position)[:2]
		
		#runs the function that retrives/runs the scan the code state True for circle scan
		#the code, False for circle the black totem
		scan_the_code = point
		
		return_array.append(scan_the_code)
		
		#print scan_the_code

		#this portion of the method gets the location of the nearest black totem
		
		#gets all of the black totems from the database
		num_of_black_totems = 1
		
		black_totems = yield self.database_query('totem_black')
		
		black_totems_poses = []

		for i in black_totems.objects:
			point = rosmsg_to_numpy(i.pose.position)[:2]
			black_totems_poses.append(point)
		
		#the follwing determins which is the closest

		#i wish python had a do while loop

		closest = black_totems_poses[0]
		dist = ((black_totems_poses[0][0]-self.pose[0][0])**2)+((black_totems_poses[0][1]-self.pose[0][1])**2)
		
		j=0 #an index for populating the dist_temp array
		while j < len(black_totems_poses):
			dist_temp = ((black_totems_poses[j][0]-self.pose[0][0])**2)+((black_totems_poses[j][1]-self.pose[0][1])**2)
			if dist_temp < dist:
				dist = dist_temp
				closest = black_totems[j]
			j+=1

		#closest now has the position of the closest black totem
		#closest is a np array

		return_array.append(closest)
		
		#returnValue has the scan the code and closest black totem location 

		defer.returnValue(return_array)
		
	@txros.util.cancellableInlineCallbacks
	def bouy_go_round(self):
		TOTEM_MARGIN = 6 #m, distance to pass behind the totem

		start_pose = self.pose[0][:2]


		locations = yield self.get_bouy_go_round_target()

		#target contains xy of target totem and unit vector from target to non target totem

		scan_the_code = locations[0]
		black_totem = locations[1]
		#an ENU vector from the scan_the_code to start pose of magnitude TOTEM_MARGIN (N=0)
		stc_waypoint = np.append((((start_pose-scan_the_code)/np.linalg.norm(start_pose-scan_the_code))*TOTEM_MARGIN)+scan_the_code, 0)

		#go to the end of that vector and look at the scan_the_code platform
		yield self.move.set_position(stc_waypoint).look_at(np.append(scan_the_code, 0)).go()
		
		#determine weather or not to circle the stc platform
		if self.get_scan_the_code() == True:
			#turn 90deg to the left so we cirlce prograde 
			yield self.move.yaw_left(math.pi/2).go()
			#we cirlce clock-wise .75 revolutions
			circle = self.move.circle_point([scan_the_code[0], scan_the_code[1], 0], "cw", .75)
		elif self.get_scan_the_code() == False:
			#an ENU vector from black_totem to self.pose of magnitude TOTEM_MARGIN (N=0)
			black_totem_waypoint = np.append(((((self.pose[0][:2]-black_totem[:2])/np.linalg.norm(self.pose[0][:2]-black_totem[:2]))*TOTEM_MARGIN)+black_totem[:2]),0)
			yield self.move.set_position(black_totem_waypoint).look_at(np.append(black_totem[:2], 0)).go()
			#turn 90deg to the right so we cirlce prograde
			yield self.move.yaw_right(math.pi/2).go()
			#we cirlce counter clock-wise .5 revolutions
			circle = self.move.circle_point([black_totem[0], black_totem[1], 0], "ccw", .5)
		yield circle.go()
		#go bakc to where was dropped off to listen for hydrophones
		yield self.move.set_position(np.append(start_pose,0)).go()
	@txros.util.cancellableInlineCallbacks
	def run (self, parameters):
		yield self.bouy_go_round()

