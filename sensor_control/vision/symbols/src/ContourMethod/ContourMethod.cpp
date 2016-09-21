#include "ContourMethod.h"
ContourMethod::ContourMethod(ros::NodeHandle& nh) :
  DockShapeVision(nh),
  blueFinder(navigator_msgs::DockShape::BLUE), 
  redFinder(navigator_msgs::DockShape::RED),
  greenFinder(navigator_msgs::DockShape::GREEN)
{
  fp.init(nh);
  ShapeDetector::init(nh);
}
void ContourMethod::GetShapes(cv::Mat &frame,navigator_msgs::DockShapes& symbols)
{
  fp.Prepare(frame);
  blueFinder.GetSymbols(fp.GetBlue(), &symbols);
  redFinder.GetSymbols(fp.GetRed(), &symbols);
  greenFinder.GetSymbols(fp.GetGreen(), &symbols);
}