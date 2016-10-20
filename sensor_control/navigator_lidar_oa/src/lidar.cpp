////////////////////////////////////////////////////////////
//
// ROS Lidar Node for RobotX
//
////////////////////////////////////////////////////////////
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <shape_msgs/SolidPrimitive.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_listener.h>
#include <navigator_msgs/PerceptionObjects.h>
#include <navigator_msgs/PerceptionObject.h>
#include <uf_common/PoseTwistStamped.h>
#include <uf_common/MoveToAction.h>
#include <actionlib/server/simple_action_server.h>

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "OccupancyGrid.h"
#include "ConnectedComponents.h"
#include "AStar.h"
#include "objects.h"
#include "bounding_boxes.h"

using namespace std;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const double MAP_SIZE_METERS = 1500.3;
const double ROI_SIZE_METERS = 201.3;
const double VOXEL_SIZE_METERS = 0.30;
const int MIN_HITS_FOR_OCCUPANCY = 50;  // 20
const int MAX_HITS_IN_CELL = 500;  // 500
const double MAXIMUM_Z_HEIGHT = 8;

float LLA_BOUNDARY_X1 = 60.87, LLA_BOUNDARY_Y1 = 11;
float LLA_BOUNDARY_X2 = -31.0, LLA_BOUNDARY_Y2 = -29.0;
float LLA_BOUNDARY_X3 = 36.0, LLA_BOUNDARY_Y3 = -163.0;
float LLA_BOUNDARY_X4 = 127.0, LLA_BOUNDARY_Y4 = -122.0;

// float LLA_BOUNDARY_X1 = -30, LLA_BOUNDARY_Y1 = 50;
// float LLA_BOUNDARY_X2 = -30, LLA_BOUNDARY_Y2 = -20;
// float LLA_BOUNDARY_X3 = 35, LLA_BOUNDARY_Y3 = -20;
// float LLA_BOUNDARY_X4 = 35, LLA_BOUNDARY_Y4 = 50;

float bounds[] = {LLA_BOUNDARY_X1, LLA_BOUNDARY_X2, LLA_BOUNDARY_X3, LLA_BOUNDARY_X4,
                  LLA_BOUNDARY_Y1, LLA_BOUNDARY_Y2, LLA_BOUNDARY_Y3, LLA_BOUNDARY_Y4};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
OccupancyGrid ogrid(MAP_SIZE_METERS, ROI_SIZE_METERS, VOXEL_SIZE_METERS, bounds);

nav_msgs::OccupancyGrid rosGrid;
geometry_msgs::Pose boatPose_enu;
geometry_msgs::Twist boatTwist_enu;
ros::Publisher pubGrid, pubMarkers, pubBuoys, pubTrajectory, pubWaypoint, pubMarkersSmall;

visualization_msgs::MarkerArray markers, small_markers;
visualization_msgs::Marker m;
ObjectTracker object_tracker;
geometry_msgs::Point waypoint_ogrid;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void cb_velodyne(const sensor_msgs::PointCloud2ConstPtr &pcloud) {
  ROS_INFO("**********************************************************");
  ROS_INFO("LIDAR | cb_velodyne...");

  // Measure elapsed time for function
  ros::Time timer = ros::Time::now();

  // Use ROS transform listener to grad up-to-date transforms between reference frames
  static tf2_ros::Buffer tfBuffer;
  static tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped T_enu_velodyne_ros;
  try {
    T_enu_velodyne_ros = tfBuffer.lookupTransform("enu", "velodyne",
                                                  ros::Time(0));  // change time to pcloud header? pcloud->header.stamp
  } catch (tf2::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }

  // Convert ROS transform to eigen transform
  Eigen::Affine3d T_enu_velodyne(Eigen::Affine3d::Identity());
  geometry_msgs::Vector3 lidarpos = T_enu_velodyne_ros.transform.translation;
  geometry_msgs::Quaternion quat = T_enu_velodyne_ros.transform.rotation;
  T_enu_velodyne.translate(Eigen::Vector3d(lidarpos.x, lidarpos.y, lidarpos.z));
  T_enu_velodyne.rotate(Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z));
  ROS_INFO_STREAM("LIDAR | Velodyne enu: " << lidarpos.x << "," << lidarpos.y << "," << lidarpos.z);

  // Update occupancy grid
  ogrid.setLidarPosition(lidarpos);
  ogrid.updatePointsAsCloud(pcloud, T_enu_velodyne, MAX_HITS_IN_CELL);
  ogrid.createBinaryROI(MIN_HITS_FOR_OCCUPANCY, MAXIMUM_Z_HEIGHT);

  // Inflate ogrid before detecting objects and calling AStar
  ogrid.inflateBinary(1);

  // Detect objects
  std::vector<objectMessage> objects;
  std::vector<std::vector<int> > cc = ConnectedComponents(ogrid, objects);

  // Publish rosgrid
  rosGrid.header.seq = 0;
  rosGrid.info.resolution = VOXEL_SIZE_METERS;
  rosGrid.header.frame_id = "enu";
  rosGrid.header.stamp = ros::Time::now();
  rosGrid.info.map_load_time = ros::Time::now();
  rosGrid.info.width = ogrid.ROI_SIZE;
  rosGrid.info.height = ogrid.ROI_SIZE;
  rosGrid.info.origin.position.x = ogrid.lidarPos.x + ogrid.ROItoMeters(0);
  rosGrid.info.origin.position.y = ogrid.lidarPos.y + ogrid.ROItoMeters(0);
  rosGrid.info.origin.position.z = ogrid.lidarPos.z;
  rosGrid.data = ogrid.ogridMap;
  pubGrid.publish(rosGrid);

  // Publish markers
  geometry_msgs::Point p;
  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker m;
  m.header.stamp = ros::Time::now();
  m.header.seq = 0;
  m.header.frame_id = "enu";

  // Erase old markers
  m.id = 1000;
  m.type = 0;
  m.action = 3;
  markers.markers.push_back(m);

  // Course Outline - change to real values or pull from service/topic
  m.id = 1001;
  m.type = visualization_msgs::Marker::LINE_STRIP;
  m.action = visualization_msgs::Marker::ADD;
  m.scale.x = 0.5;

  p.x = LLA_BOUNDARY_X1;
  p.y = LLA_BOUNDARY_Y1;
  p.z = lidarpos.z;
  m.points.push_back(p);
  p.x = LLA_BOUNDARY_X2;
  p.y = LLA_BOUNDARY_Y2;
  p.z = lidarpos.z;
  m.points.push_back(p);
  p.x = LLA_BOUNDARY_X3;
  p.y = LLA_BOUNDARY_Y3;
  p.z = lidarpos.z;
  m.points.push_back(p);
  p.x = LLA_BOUNDARY_X4;
  p.y = LLA_BOUNDARY_Y4;
  p.z = lidarpos.z;
  m.points.push_back(p);
  p.x = LLA_BOUNDARY_X1;
  p.y = LLA_BOUNDARY_Y1;
  p.z = lidarpos.z;
  m.points.push_back(p);
  m.color.a = 0.6;
  m.color.r = 1;
  m.color.g = 1;
  m.color.b = 1;
  markers.markers.push_back(m);

  // Publish buoys
  navigator_msgs::PerceptionObjects allBuoys;
  navigator_msgs::PerceptionObject buoy;
  geometry_msgs::Point32 p32;
  buoy.header.seq = 0;
  buoy.header.frame_id = "enu";
  buoy.header.stamp = ros::Time::now();

  auto object_permanence = object_tracker.add_objects(objects, boatPose_enu);
  std::vector<objectMessage> small_objects =
      BoundingBox::get_accurate_objects(pcloud, object_permanence, T_enu_velodyne);
  int max_id = 0;

  for (auto obj : object_permanence) {
    buoy.header.stamp = ros::Time::now();
    buoy.type = navigator_msgs::PerceptionObject::UNKNOWN;
    buoy.id = obj.id;
    buoy.confidence = 0;
    buoy.position = obj.position;
    buoy.height = obj.scale.z;
    buoy.width = obj.scale.x;
    buoy.depth = obj.scale.y;
    buoy.points = obj.beams;
    allBuoys.objects.push_back(buoy);

    // Buoys as markers
    visualization_msgs::Marker m2;
    m2.header.stamp = ros::Time::now();
    m2.header.seq = 0;
    m2.header.frame_id = "enu";
    m2.header.stamp = ros::Time::now();
    m2.id = obj.id;
    m2.type = visualization_msgs::Marker::CUBE;
    m2.action = visualization_msgs::Marker::ADD;
    m2.pose.position = obj.position;
    m2.scale = obj.scale;
    m2.color.a = 0.6;
    m2.color.r = 1;
    m2.color.g = 1;
    m2.color.b = 1;
    markers.markers.push_back(m2);
    if (m2.id > max_id) max_id = m2.id;

    visualization_msgs::Marker m4;
    m4.header.stamp = ros::Time::now();
    m4.header.seq = 0;
    m4.header.frame_id = "enu";
    m4.header.stamp = ros::Time::now();
    m4.id = obj.id + 10000;
    m4.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    m4.action = visualization_msgs::Marker::ADD;
    m4.pose.position.x = obj.position.x;
    m4.pose.position.y = obj.position.y;
    m4.pose.position.z = obj.position.z + 1.0;
    m4.scale.x = 1.0;
    m4.scale.y = 1.0;
    m4.scale.z = 1.0;
    m4.color.a = 0.6;
    m4.color.r = 1;
    m4.color.g = 0;
    m4.color.b = 0;
    m4.text = std::to_string(obj.id);
    markers.markers.push_back(m4);
  }
  pubMarkers.publish(markers);
  pubBuoys.publish(allBuoys);

  visualization_msgs::Marker m3;
  small_markers.markers.clear();
  m3.header.seq = 0;
  m3.header.frame_id = "enu";
  m3.action = 3;
  for (auto obj : small_objects) {
    m3.header.stamp = ros::Time::now();
    m3.id = obj.id;
    m3.type = visualization_msgs::Marker::CUBE;
    m3.action = visualization_msgs::Marker::ADD;
    m3.pose.position = obj.position;
    m3.scale = obj.scale;
    m3.color.a = 0.6;
    m3.color.r = 1;
    m3.color.g = 1;
    m3.color.b = 1;
    small_markers.markers.push_back(m3);
  }
  pubMarkersSmall.publish(small_markers);

  // Elapsed time
  ROS_INFO_STREAM("LIDAR | Elapsed time: " << (ros::Time::now() - timer).toSec());
  ROS_INFO("**********************************************************");
}

void cb_odom(const nav_msgs::OdometryConstPtr &odom) {
  // ROS_INFO("cb_odom...");
  boatPose_enu = odom->pose.pose;
  boatTwist_enu = odom->twist.twist;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[]) {
  // Ros init
  ros::init(argc, argv, "lidar");
  ros::Time::init();

  // ros::init(argc, argv, "lidar", ros::init_options::AnonymousName);

  // Check that ROS is alive before continuing... After 10 minutes quit!
  ROS_INFO("LIDAR | Checking ROS master is alive...");
  ros::Time timer = ros::Time::now();
  while (!ros::master::check()) {
    if ((ros::Time::now() - timer).toSec() > 600) {
      return -1;
    }
    ros::Duration(0.1).sleep();
  }
  ROS_INFO_STREAM("ROS Master: " << ros::master::getHost());

  // Node handler
  ros::NodeHandle nh;

  // Subscribe to odom and the velodyne
  ros::Subscriber sub1 = nh.subscribe("/velodyne_points", 1, cb_velodyne);
  ros::Subscriber sub2 = nh.subscribe("/odom", 1, cb_odom);

  // Publish occupancy grid and visualization markers
  pubGrid = nh.advertise<nav_msgs::OccupancyGrid>("ogrid_batcave", 10);
  pubMarkers = nh.advertise<visualization_msgs::MarkerArray>("/unclassified/objects/markers", 10);
  pubBuoys = nh.advertise<navigator_msgs::PerceptionObjects>("/unclassified/objects", 10);

  pubMarkersSmall = nh.advertise<visualization_msgs::MarkerArray>("/unclassified/objects/markers_accurate", 10);

  // Give control to ROS
  ros::spin();

  return 0;
}
