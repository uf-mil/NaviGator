#pragma once
#include <ros/ros.h>
#include <navigator_msgs/ObjectDBQuery.h>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <nav_msgs/OccupancyGrid.h>
#include "opencv2/opencv.hpp"



class FakeOgrid {
private:
    ros::NodeHandle *nh;
    ros::Rate loopRate;
    ros::ServiceClient getDataBase;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    ros::Publisher pubMarkers;
    nav_msgs::OccupancyGrid rosGrid;
    const double VOXEL_SIZE_METERS = 0.30;
    const int MIN_HIT = 1;
    const float BOX_SIZE = 0.5;
    ros::Publisher pubGrid;

    union floatConverter {
        float f;
        struct {
            uint8_t data[4];
        };
    };

    struct rectangle {
        float x, y, width, height;
    };

    struct point { //geometry msgs don't have damn constructors
        float x, y, z;
    };
    float dot(point &a, point &b);

    bool insideRect(float cX, float cY, rectangle rect, bool f);

    message_filters::Subscriber<sensor_msgs::PointCloud2> lidarSub;
    message_filters::Cache<sensor_msgs::PointCloud2> lidarCache;

    visualization_msgs::Marker drawSquareMarker(int x, int y, int size);

public:
    bool running;
    FakeOgrid(ros::NodeHandle &nh);
    void drawOgrid();

};