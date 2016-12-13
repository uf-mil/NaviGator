#include "dock_service.hpp"

DockService::DockService(ros::NodeHandle &nh, FakeOgrid &fakeOgrid) {
    this->nh = &nh;
    this->fakeOgrid = &fakeOgrid;
    getBaysService = nh.advertiseService("/identify_dock/active", &DockService::getBaysCallback, this);
    std::cout << "Service Running" << std::endl;
}

bool DockService::getBaysCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
    std::cout << "Recieved service" << std::endl;
    fakeOgrid->running = req.data;
    res.success = true;
    res.message = "Set";
    return true;
}
