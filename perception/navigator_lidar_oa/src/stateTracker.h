#ifndef STATETRACKER_H
#define STATETRACKER_H

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>

struct rpy
{
	rpy(double t_, std::vector<double> rpy_) : t(t_) {
		roll = rpy_[0];
		pitch = rpy_[1];
		yaw = rpy_[2];
	}
	double t,roll,pitch,yaw;
};

class stateTracker
{
public:
	stateTracker() = default;
	void update(ros::Time time, Eigen::Matrix3d mat) 
	{
		double roll,pitch,yaw;
		roll = atan2(-mat(1,2), mat(2,2) );    
		double sr = sin( roll ), cr = cos( roll );
		pitch = atan2( mat(0,2),  cr*mat(2,2) - sr*mat(1,2) );
		yaw = atan2( -mat(0,1), mat(0,0) ); 	
		ROS_INFO_STREAM("LIDAR | BOAT XYZ Rotation: " << roll*180/M_PI << "," << pitch*180/M_PI << "," << yaw*180/M_PI );
		states.push_back(rpy(time.toSec(),{roll,pitch,yaw}));
		if (states.size() > 1) {
			auto last = states.size()-1;
			auto tdiff = states[last].t - states[last-1].t;
			roll = (states[last].roll - states[last-1].roll) / (tdiff);
			pitch = (states[last].pitch - states[last-1].pitch) / (tdiff);
			yaw = (states[last].yaw - states[last-1].yaw) / (tdiff);
			states_dot.push_back(rpy(time.toSec(),{roll,pitch,yaw}));
		} else {
			states_dot.push_back(rpy(time.toSec(),{0,0,0}));
		}
	}
	std::deque<rpy> states;
	std::deque<rpy> states_dot;
};



#endif