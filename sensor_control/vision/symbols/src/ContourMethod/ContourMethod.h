#pragma once
#include "../DockShapeVision.h"
#include "FrameProc.h"
#include "ShapeFind.h"
#include <navigator_msgs/DockShapes.h>
class ContourMethod : public DockShapeVision
{
  ShapeFind blueFinder;
  ShapeFind redFinder;
  ShapeFind greenFinder;
  FrameProc fp;
  public:
    ContourMethod(ros::NodeHandle& nh);
    void GetShapes(cv::Mat &frame,navigator_msgs::DockShapes& symbols);
};