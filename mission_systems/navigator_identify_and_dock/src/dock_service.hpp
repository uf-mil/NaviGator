#pragma once
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include "fake_ogrid.hpp"

class DockService {
private:
    ros::NodeHandle *nh;
    ros::ServiceServer getBaysService;
    FakeOgrid *fakeOgrid;

public:
    DockService(ros::NodeHandle &nh, FakeOgrid &fakeOgrid);
    bool getBaysCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

};