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
class StartGateMarshall(Navigator):
	"""this method gets the location of the nearest black totem and the scan the code"""
	
	#runs the scan the code preception script eventually
	def get_scan_the_code(self):
		return False

	#returns the xy of target totem and unit vector from target to non target totem
	@txros.util.cancellableInlineCallbacks
	def get_bouy_go_round_target(self):
		return_array = []
		
		#gets the xy and state of the scan the code from the database
		scan_the_code = np.array([])
		query = 'stc_platform'
		res = yield self.database_query(query)
		#makes sure that only 1 scan the code exists
		assert len(res.objects) == 1
		if not res.found:
			raise TaskException(query + ' not found in object database')
		point = rosmsg_to_numpy(res.objects[0].pose.position)[:2]
		
		#runs the function that retrives/runs the scan the code state True for circle scan
		#the code, False for circle the black totem
		scan_the_code = point
		
		return_array.append(scan_the_code)
		
		#print scan_the_code

		"""this portion of the method gets the location of the nearest black totem"""
		
		#gets all of the black totems from the database
		num_of_black_totems = 1
		
		black_totems = yield self.database_query('totem_black')
		
		black_totems_poses = []

		for i in black_totems.objects:
			point = rosmsg_to_numpy(i.pose.position)[:2]
			black_totems_poses.append(point)
		#print black_totems_poses
		#determins which is the closest
		closest = black_totems_poses[0]
		dist = math.sqrt(((black_totems_poses[0][0]-self.pose[0][0])**2)+((black_totems_poses[0][1]-self.pose[0][1])**2))
		
		j=0
		while j < len(black_totems_poses):
			dist_temp = math.sqrt(((black_totems_poses[j][0]-self.pose[0][0])**2)+((black_totems_poses[j][1]-self.pose[0][1])**2))
			if dist_temp < dist:
				dist = dist_temp
				closest = black_totems[j]
			j+=1
		#closest now has the position of the closest black totem
		#closest is a np array

		#hard code values for before database works
		#closest = np.array([30+0,10-30])

		return_array.append(closest)
		
		defer.returnValue(return_array)
		
	
	@txros.util.cancellableInlineCallbacks
	def bouy_go_round(self):
		TOTEM_MARGIN = 6 #m, distance to pass behind the totem
		
		WAYPOINT_RANGE = math.pi #the amount of radians to go around the bouy
		WAYPOINT_RES = 4 #number of waypoints+1 to have evenly distributed behind and around the bouy
		
		STC_DIST = 4.5 #m, distance to stay away from scan the code when scanning code

		start_pose = self.pose[0][:2]

		locations = yield self.get_bouy_go_round_target()

		#target contains xy of target totem and unit vector from target to non target totem

		scan_the_code = locations[0]
		closest = locations[1]
		way_point_zero_stc = np.append((((start_pose-scan_the_code)/np.linalg.norm(start_pose-scan_the_code))*STC_DIST)+scan_the_code, 0)

		yield self.move.set_position(way_point_zero_stc).look_at(np.append(scan_the_code, 0)).go()				
		print way_point_zero_stc

		target_data = np.array([])


		delta_theta = WAYPOINT_RANGE/WAYPOINT_RES

		if self.get_scan_the_code() == True:
			target_data = np.append(target_data, scan_the_code[:2])
			unit_vector = ((np.array(closest-(scan_the_code[:2])))/(np.linalg.norm(closest-(scan_the_code[:2]))))
			target_data = np.append(target_data, unit_vector)
			delta_theta = (WAYPOINT_RANGE/WAYPOINT_RES)

		
		
		elif self.get_scan_the_code() == False:
			target_data = np.append(target_data, closest)
			unit_vector = ((np.array((scan_the_code[:2])-closest))/(np.linalg.norm((scan_the_code[:2])-closest)))
			target_data = np.append(target_data, unit_vector)
			delta_theta = -(WAYPOINT_RANGE/WAYPOINT_RES)
		

		#print start_pose
		
		inside_unit_vector = target_data[2:]
		
		#delta_theta = WAYPOINT_RANGE/WAYPOINT_RES
		
		waypoints=[]
		#getting angle of unit vector
		#this should prbably be a function
		#if vector is in quadrant 1
		if inside_unit_vector[0]>0 and inside_unit_vector[1]>0:
			theta = math.atan(inside_unit_vector[1]/inside_unit_vector[0])
		#if vector is in  quadrant 2
		elif inside_unit_vector[0]<0 and inside_unit_vector[1]>0:
			theta = math.pi - math.atan(inside_unit_vector[1]/(-1*inside_unit_vector[0]))
		#if vector is in quadrant 3
		elif inside_unit_vector[0]<0 and inside_unit_vector[1]<0:
			theta = math.pi + math.atan(inside_unit_vector[1]/inside_unit_vector[0])
		#if vector is in quadrant 4
		elif inside_unit_vector[0]>0 and inside_unit_vector[1]<0:
			theta = 2*math.pi - math.atan((-1*inside_unit_vector[1])/inside_unit_vector[0])
		



		#creating waypoints around the back of the bouy
		for i in range(WAYPOINT_RES+1):
			waypoints.append(np.array([math.cos(theta-(i*delta_theta))*TOTEM_MARGIN, math.sin(theta-(i*delta_theta))*TOTEM_MARGIN])+(target_data[:2]))
			print 
			print i
			print waypoints[i]
			print (theta-(i*delta_theta))*(180/math.pi)
			print 
		for i in range(WAYPOINT_RES):
			yield self.move.set_position(np.append(waypoints[i],0)).look_at(np.append(waypoints[i+1],0)).go()
		
		yield self.move.set_position(np.append(waypoints[WAYPOINT_RES],0)).look_at(np.append(start_pose, 0)).go()
		#yield self.move.set_position(np.append(waypoints[WAYPOINT_RES],0)).go()

		way_point_one_inside = np.array((inside_unit_vector*TOTEM_MARGIN)+target_data[:2])
		way_point_one_inside = np.append(way_point_one_inside,0)
		#self.send_feedback (way_point_one_inside)
		#print way_point_one_inside

		unit_vector_target_to_boat = (target_data[:2]-start_pose)/np.linalg.norm(target_data[:2]-start_pose)
		way_point_two_backside = (unit_vector_target_to_boat*TOTEM_MARGIN)+target_data[:2]
		way_point_two_backside = np.append(way_point_two_backside, 0)
		#self.send_feedback (way_point_two_backside)
		#print way_point_two_backside

		way_point_three_outside = (-inside_unit_vector*TOTEM_MARGIN)+target_data[:2]
		way_point_three_outside = np.append(way_point_three_outside,0)
		#self.send_feedback (way_point_three_outside)
		#print way_point_three_outside

		

		#yield self.move.set_position(way_point_one_inside).look_at(way_point_two_backside).go()


		#yield self.move.set_position(way_point_two_backside).look_at(way_point_three_outside).go()

		
		#yield self.move.set_position(way_point_three_outside).look_at(np.append(start_pose,0)).go()


		yield self.move.set_position(np.append(start_pose,0)).go()

		
	@txros.util.cancellableInlineCallbacks
	def run (self, parameters):
		yield self.bouy_go_round()














