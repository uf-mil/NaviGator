#include <ros/ros.h>
#include <iostream>
#include "dock_service.hpp"
#include "fake_ogrid.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "identify_and_dock");
    ros::NodeHandle nh;

    FakeOgrid fakeOgrid(nh);
    DockService dockService(nh, fakeOgrid);
    fakeOgrid.drawOgrid();
    ros::spin();

}