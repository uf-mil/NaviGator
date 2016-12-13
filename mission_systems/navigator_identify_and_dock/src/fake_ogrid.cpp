#include "fake_ogrid.hpp"

FakeOgrid::FakeOgrid(ros::NodeHandle &nh) : loopRate(2), lidarSub(nh, "/velodyne_points", 10),
    lidarCache(lidarSub, 1), tfBuffer(), tfListener(tfBuffer, nh)  {
    this->nh = &nh;
    running = false;
    getDataBase = nh.serviceClient<navigator_msgs::ObjectDBQuery>("/database/requests");
    pubMarkers = nh.advertise<visualization_msgs::MarkerArray>("dock_ogrid_markers", 10, true);
    pubGrid = nh.advertise<nav_msgs::OccupancyGrid>("/mission_ogrid", 10, true);
    srand(std::time(NULL));
}

float FakeOgrid::dot(point &a, point &b) {
    return a.x * b.x + a.y * b.y;
}

bool FakeOgrid::insideRect(float cX, float cY, const rectangle rect, bool f = false) {
    if (cX < rect.x || cX > rect.x + rect.width || cY < rect.y || cY > rect.y + rect.height) {
        return false;
    }
    return true;
}
visualization_msgs::Marker FakeOgrid::drawSquareMarker(int x, int y, int size) {
    visualization_msgs::Marker marker_point;
    marker_point.header.seq = 0;
    marker_point.header.frame_id = "enu";
    marker_point.id = rand() % 100000000;
    std::cout << marker_point.id << std::endl;
    marker_point.type = visualization_msgs::Marker::CUBE;
    marker_point.pose.position.x = x;
    marker_point.pose.position.y = y;
    marker_point.pose.position.z = 0;
    marker_point.scale.x = size;
    marker_point.scale.y = size;
    marker_point.scale.z = 0;
    marker_point.color.a = 1.0;
    marker_point.color.r = 1.0;
    marker_point.color.g = 0.0;
    marker_point.color.b = 0.0;
    return marker_point;
}
void FakeOgrid::drawOgrid() {
    while (ros::ok()) {
        if (!running) {
            ros::spinOnce();
            loopRate.sleep();
            continue;
        }

        //Get Dock from Database
        navigator_msgs::ObjectDBQuery dataBaseSrv;
        dataBaseSrv.request.name = "dock";
        if (!getDataBase.call(dataBaseSrv)) {
            std::cout << "Can't find DOCK in database" << std::endl;
            running = false;
            continue;
        }

        sensor_msgs::PointCloud2ConstPtr scloud = lidarCache.getElemBeforeTime(ros::Time::now());


        if (!scloud) {
            std::cout << "Can't find POINTCLOUD" << std::endl;
            running = false;
            continue;
        }

        if (running) {
            visualization_msgs::MarkerArray markers;

            const rectangle searchArea = {(float)dataBaseSrv.response.objects[0].position.x - 10, (float)dataBaseSrv.response.objects[0].position.y - 10, 20.f, 20.f};

            geometry_msgs::TransformStamped t_enu_vel;

            try {
                t_enu_vel = tfBuffer.lookupTransform("enu", "velodyne", ros::Time(0));
            }
            catch (tf2::TransformException &ex) {
                ROS_ERROR("%s", ex.what());
            }

            sensor_msgs::PointCloud2 *cloud_transformed;
            tf2::doTransform(*scloud, *cloud_transformed, t_enu_vel);

            pcl::PointCloud<pcl::PointXYZ> cloud;
            pcl::fromROSMsg(*cloud_transformed, cloud);

            cv::Mat dockOgridDraw = cv::Mat::zeros((int)(searchArea.height / VOXEL_SIZE_METERS), (int)(searchArea.width / VOXEL_SIZE_METERS), CV_8U);

            std::vector<std::vector<int> > test ((int)(searchArea.width / BOX_SIZE), std::vector<int> ((int)(searchArea.height / BOX_SIZE), 0));

            for (unsigned index = 0; index < cloud.size(); index++)
            {
                pcl::PointXYZ& p = cloud[index];
                if (insideRect(p.x, p.y, searchArea)) {

                    for (int i = 0; i < (int)(searchArea.width / BOX_SIZE); i++) {
                        for (int j = 0; j < (int)(searchArea.height / BOX_SIZE); j++) {
                            const rectangle gridS = { searchArea.x + BOX_SIZE * i, searchArea.y + BOX_SIZE * j, BOX_SIZE , BOX_SIZE};
                            if (insideRect(p.x, p.y, gridS, true)) {
                                test.at(i).at(j) += 1;
                            }
                        }
                    }
                }

            } // END OF PARSING POINTCLOUD


            //Draw fake ogrid
            for (int i = 0; i < test.size(); i++) {
                for (int j = 0; j < test[i].size(); j++) {
                    if (test.at(i).at(j) > MIN_HIT) {
                        const rectangle gridS = { BOX_SIZE * i,  BOX_SIZE * j, BOX_SIZE , BOX_SIZE};
                        cv::rectangle(dockOgridDraw, cv::Rect((int)(gridS.x / VOXEL_SIZE_METERS), (int)(gridS.y / VOXEL_SIZE_METERS), (int)(BOX_SIZE / VOXEL_SIZE_METERS), (int)(BOX_SIZE / VOXEL_SIZE_METERS)), cv::Scalar(255), -1, 8);
                    }
                }
            }
            // cv::imshow("sds", dockOgridDraw);
            // cv::waitKey(0);
            cv::transpose(dockOgridDraw, dockOgridDraw);
            cv::flip(dockOgridDraw, dockOgridDraw, 1);


            std::vector<int8_t> data;
            for (int i = dockOgridDraw.cols - 1; i >= 0; i--) {
                for (int j = 0; j < dockOgridDraw.rows; j++) {
                    if ((int)dockOgridDraw.at<uint8_t>(cv::Point(i, j)) > 200) {
                        data.push_back(100);
                    }
                    else data.push_back(0);
                }
            }

            rosGrid.header.seq = 0;
            rosGrid.info.resolution = VOXEL_SIZE_METERS;
            rosGrid.header.frame_id = "enu";
            rosGrid.header.stamp = ros::Time::now();
            rosGrid.info.map_load_time = ros::Time::now();
            rosGrid.info.width = dockOgridDraw.rows;
            rosGrid.info.height = dockOgridDraw.rows;
            rosGrid.info.origin.position.x = searchArea.x;
            rosGrid.info.origin.position.y = searchArea.y;
            rosGrid.info.origin.position.z = 0;
            rosGrid.data = data;
            pubGrid.publish(rosGrid);

        }
        running = true;
        ros::spinOnce();
        loopRate.sleep();
    }


}