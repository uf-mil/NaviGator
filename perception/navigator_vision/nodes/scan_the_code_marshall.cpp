#include "ros/ros.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cstring>
#include <cmath>
#include <iostream>
#include <fstream>
#include <pcl/filters/extract_indices.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <mil_msgs/ObjectDBQuery.h>

//this is the Lidar Analyzer class. Only one is every created.
#define _USE_MATH_DEFINES
class LidarAnalyzer
{
public:
  tf::TransformListener enuToVelodyneListener;//tf listener for setRvizPointCallback
  geometry_msgs::PointStamped stcCenterEnu;
  ros::ServiceClient service_client_;
  
  //tf::TransformBroadcaster velodyneToVelodyneMarshallBroadcaster; 
  //tf::Transform velodyneToVelodyneMarshallTransform;
  LidarAnalyzer()
  {
    service_client_ = n_.serviceClient<mil_msgs::ObjectDBQuery>("/database/requests");

    // TODO: do this only when
    mil_msgs::ObjectDBQuery::Request req;
    req.name = "stc_platform";
    mil_msgs::ObjectDBQuery::Response res;
    if (!service_client_.call(req, res)) throw std::runtime_error("bla2");
    if (res.objects.size() == 0) throw std::runtime_error("bla");
    stcCenterEnu.point = res.objects[0].pose.position;
    stcCenterEnu.header = res.objects[0].header;
    // END TODO block


    pub_debug_points = n_.advertise<sensor_msgs::PointCloud2>("stc_led_pts_marshall", 1000);

    sub_velodyne_points = n_.subscribe("velodyne_points", 1000, &LidarAnalyzer::pointCloudAnalysisCallback, this);

    //sub_rviz_point = n_.subscribe("clicked_point",1000, &LidarAnalyzer::rvizPointSetCallback, this);
   
  }

  void pointCloudAnalysisCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
  {
    geometry_msgs::PointStamped stcCenterVelodyne;
    try
    {
      // ENU is global, so time can be anything
      stcCenterEnu.header.stamp = input->header.stamp;
      this->enuToVelodyneListener.waitForTransform(input->header.frame_id, stcCenterEnu.header.frame_id , input->header.stamp, ros::Duration(10.0));
      this->enuToVelodyneListener.transformPoint(input->header.frame_id, stcCenterEnu, stcCenterVelodyne);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("Received an exception trying to transform a point in rvizPointSetCallback: %s", ex.what());
      return;
    }

    stcCenter[0] = stcCenterVelodyne.point.x;
    stcCenter[1] = stcCenterVelodyne.point.y;
    stcCenter[2] = stcCenterVelodyne.point.z;

    //initialize pcl_pc2
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);//saves input as pcl_pc2
    //initialize pt_cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *pt_cloud);
    this->cloudOfIntrest = pt_cloud;
  
    //TODO make service call to pcodar for center of stc_platform in the RF of the velodyne
    //if we dont have the point from RVIZ yet, kill the callback and well see if we have one later.
    if (stcCenter==NULL)
    {
      ROS_ERROR("no STC center yet, cant do no math:-(, please feed me an rviz point");
      return;
    }
    ROS_INFO("cloudOfIntrest %i", cloudOfIntrest->size());  
    this->cylinderPrune(this->radius, this->stcCenter, this->cloudOfIntrest);
    ROS_INFO("cyindar prune finished");
    this->cloudRotateZ(this->stcCenter,this->cloudOfIntrest, -1);
    ROS_INFO("rotation finfished");
    this->zPartitioner(cloudOfIntrest, .1);
    ROS_INFO("zPartition finfished");
    this->lm(this->cloudRefined);
    ROS_INFO("lm finished"); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr ledPts(new pcl::PointCloud<pcl::PointXYZ>);     
    if (this->cloudRefined->size()<=1)
    {
      ROS_ERROR("too spooky! 0 or 1 partition(s). nonupdating stcCenters is known to cause this error");
      return;
    }
    ledPts = this->calcLedPts();
    ROS_INFO("calcLedPts finished"); 
     
    this->cloudRotateZ(this->stcCenter,ledPts, 1);//now we rotate back
    //this->cloudRotateZ(this->stcCenter,this->cloudOfIntrest, 1);
    ROS_INFO("rotation back finfished\n\n"); 
    //ledPts = cloudOfIntrest;
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*ledPts, output_msg);
    //pcl::toROSMsg(*cloudRefined, output_msg);
    
    output_msg.header = input->header;
    pub_debug_points.publish(output_msg);
    ledPts->clear();
    partitions.erase(partitions.begin(),partitions.end());
    return;
  }

/*
  void rvizPointSetCallback(const geometry_msgs::PointStamped::ConstPtr& stcCenterENU)//this works half of time
  //errors out the other half in simulation due to
  //Lookup would require extrapolation into the past.
  {
    ROS_INFO("GOT RVIZ POINT");
    tf::StampedTransform transform;
    
    try
    {
      this->enuToVelodyneListener.waitForTransform("/velodyne","/enu",stcCenterENU->header.stamp, ros::Duration(10.0));
      this->enuToVelodyneListener.transformPoint("/velodyne", *stcCenterENU, stcCenterVelodyne);
    }

    catch (tf::TransformException &ex)
    {
      //ros::Duration(10.0).sleep();
      
      ROS_ERROR("Received an exception trying to transform a point in rvizPointSetCallback: %s", ex.what());
      return;
    }
    ROS_INFO("rviz point is now in velodyne frame\n x: %f\ny: %f\nz: %f", stcCenterVelodyne.point.x, stcCenterVelodyne.point.y, stcCenterVelodyne.point.z);
  stcCenter = new float [3];
  stcCenter[0] = stcCenterVelodyne.point.x;
  stcCenter[1] = stcCenterVelodyne.point.y;
  stcCenter[2] = stcCenterVelodyne.point.z;
  }
*/

private:
  ros::NodeHandle n_;
  ros::Publisher pub_debug_points;
  ros::Subscriber sub_velodyne_points, sub_rviz_point;
  int count;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOfIntrest, cloudRefined;
 
  //pcl::PointCloud<pcl::PointXYZ>::Ptr ledPts(new pcl::PointCloud<pcl::PointXYZ>);

  float * betaHat = new float [2];
  float stcCenter[3];
  const float radius = 1;//
  const float ledHeight = .7;//tallness of the LED Pannel
  const float ledFrac = .5;//the fraction of the surface as a whole that the led panel covers 
  
 

  //tf::TransformBroadcaster br;
  //tf::Transform transform; 
  
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> partitions;
  //function to erase ponts outside of a specified vertical cylendar
  void cylinderPrune
  (
  float radius,
  float center[3],
  pcl::PointCloud<pcl::PointXYZ>::Ptr input
  )
  {
    float dist = pow(radius,2);
    pcl::PointIndices::Ptr badPtsIndx(new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    for (size_t i=0; i< input->size(); ++i)
    {
      pcl::PointXYZ pt =  input->points[i];
      float distTemp = (pow(pt.x-center[0],2))+(pow(pt.y-center[1],2));
     
      if (distTemp>dist)
      {
        badPtsIndx->indices.push_back(i);
      }
    }
    extract.setInputCloud(input);
    extract.setIndices(badPtsIndx);
    extract.setNegative(true);
    extract.filter(*input);
  }

  //rotates Cloud of intrest about the z axis so that stc center lines on the x axis
  //this will make it easier to analyze
  void cloudRotateZ
  (
  float center[3],
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
  int dir
  )
  {
    float theta = 0;  
    //for quadrant 1:
    if (center[0]>0&&center[1]>0)
      theta = atan(abs(center[1]/center[0]));
    //for quadrant 2:
    if (center[0]<0&&center[1]>0)
      theta = atan(abs(center[1]/center[0]))+(M_PI/2);
    //for quadrant 3:
    if (center[0]<0&&center[1]<0)
      theta = atan(abs(center[1]/center[0]))+(M_PI);
    //for quadrant 4:
    if (center[0]>0&&center[1]>0)
      theta = atan(abs(center[1]/center[0]))+(3*M_PI/2);
      
    //now we iterate through the point cloud and rotate each point around the z axis by -theta radians
    for (size_t i=0;i<cloud->size();++i)
    {
      float phi = atan(cloud->points[i].y/cloud->points[i].x);
      float r = sqrt(pow(cloud->points[i].y,2)+pow(cloud->points[i].x,2));
      phi+=(theta*dir);
      cloud->points[i].x = r*cos(phi);//TODO should the cos sin be other way around
      cloud->points[i].y = r*sin(phi);
    }
  }

   
  
  //function designed to partition points on the 2d (z,y) based on the bars running parelles to the y axis
  void zPartitioner(pcl::PointCloud<pcl::PointXYZ>::Ptr master, float bubble)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr local (new pcl::PointCloud<pcl::PointXYZ>);
    //copyPointCloud (*master, *local);
    //copying the data from masterPtr to local for mutilation
    local->width = master->width;
    local->height = master->height;
    local->points.resize(local->width*local->height);
    //ROS_INFO("populating local with master");
    for (size_t i=0; i<master->size();++i)
    {
      local->points[i]=master->points[i];
    }
    
    //ROS_INFO("master length %i	local length %i\n", master->size(),local->size());  
    //ROS_INFO("finished populating local with master");

    //ROS_INFO("popualting the partitions");

    for (size_t n=0;n<local->size();++n)
    {
      float lowerBound = local->points[n].z-(bubble/2); 
      float upperBound = local->points[n].z+(bubble/2); 
      pcl::PointCloud<pcl::PointXYZ>::Ptr singlePartition (new pcl::PointCloud<pcl::PointXYZ>);
      //ROS_INFO("grabed root point for partition %i", partitions.size());
      for (size_t i = 0;i<local->size();)
      {
        //ROS_INFO("looking at point %i of %i", i,local->size());
        if ((local->points[i].z>lowerBound)&&(local->points[i].z<upperBound))//if we find a point
        {//in the z range
          singlePartition->points.push_back(local->points[i]);//we add that point to singlePartition
          if ((local->points[i].z-(bubble/2))<lowerBound)
            lowerBound = (local->points[i].z-(bubble/2));//we reset bounds
          if ((local->points[i].z+(bubble/2))>upperBound)
            upperBound = (local->points[i].z+(bubble/2));
          //pcl::PointCloud::<PointXYZ>::iterator del = local->points[i];
          local->points.erase(local->points.begin()+i);//why no work
          local->width--;
          //local->height--; // maybe, hard maybe
          local->points.resize(local->width*local->height);
        }
        else
          ++i;//if that point dont work, we simply incriment and go to the next point
      }
      ROS_INFO("AHHHHHHH: %i", local->size());
      partitions.push_back(singlePartition);//we add single partition to partitions
      //local->points.resize(local->width*local->height);
      //ROS_INFO("populated partition with  %i points", n); 
      
      //delete &goodPtsIndx;
      //delete &singlePartition;
    }
    local->clear();
    //delete &local;

    //ROS_INFO("finding avgXYZ of the partitions"); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr avgOfPartitions ( new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i=0; i<partitions.size();++i)
    {
      pcl::PointXYZ avgXYZ;
      //ROS_INFO("finding avgXYZ for partition %i", i);
      for (size_t j=0; j<partitions.at(i)->size(); ++j)
      {
        avgXYZ.x+=partitions.at(i)->points[j].x;
        avgXYZ.y+=partitions.at(i)->points[j].y;
        avgXYZ.z+=partitions.at(i)->points[j].z;
      }
      avgXYZ.x/=partitions.at(i)->size();
      avgXYZ.y/=partitions.at(i)->size();
      avgXYZ.z/=partitions.at(i)->size();
      avgOfPartitions->points.push_back(avgXYZ);
    }
    
    //now we order the partition vector from lowest avg z to highest avg z
    //pcl::PointCloud<pcl::PointXYZ>::Ptr avgOfPartitionsOrdered (new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i=0;i<avgOfPartitions->size();++i)
    {
      int lowestZIdx=i;
      for(size_t j=i;j<avgOfPartitions->size()-i;++j)
      {
        if (avgOfPartitions->points[j].z<avgOfPartitions->points[i].z)
        {
          lowestZIdx = j;
        }
      
      }
      //swap point[i] and point [j] in avgOfPartitions
      pcl::PointXYZ swapPoint= avgOfPartitions->points[lowestZIdx];
      avgOfPartitions->points[lowestZIdx] = avgOfPartitions->points[i];
      avgOfPartitions->points[i] = swapPoint;
      //swap i and j clouds in partitions
      std::swap(partitions.at(lowestZIdx),partitions.at(i));
    }

    //ROS_INFO("finished finding avgXYZ of the partitions"); 
    cloudRefined = avgOfPartitions;
    return;
  }

  //function to build the linear model based on y and z qualities of a collection of points
  void lm(pcl::PointCloud<pcl::PointXYZ>::Ptr input)
  {
    std::ofstream data;
    data.open("data.txt");
    Eigen::MatrixXd X(input->size(), 2), Y(input->size(), 1), _betaHat(2,1);
    for(size_t i=0; i< input->size();++i)
    {

      X.row(i)<<1,input->points[i].z;
      Y.row(i)<<input->points[i].y;
      data<<input->points[i].z<<"	"<<input->points[i].y<<"\n";
    }
    
    _betaHat = (X.transpose() * X).ldlt().solve(X.transpose() * Y); 
    data<<"set size ratio -1\n";
    data<<"f(x)="<<_betaHat(0,0)<<"+"<<_betaHat(1,0)<<"*x\n";
    betaHat[0] = _betaHat(0,0);
    betaHat[1] = _betaHat(1,0);
    data.close();
  }
  //lease quares line equation from lm
  float  lmFOfx(float x)
  {
    float y;
    y = this->betaHat[0]+(this->betaHat[1]*x);
    return y;
  };
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr calcLedPts()
  {
    //first we find the most Z pts of the cloudRefined
    pcl::PointXYZ mostZPt = cloudRefined->points[0];
    ROS_INFO("spooky scarry: %i", cloudRefined->size());
    for(size_t i = 0;i<cloudRefined->size();++i)
    {
      if (cloudRefined->points[i].z>mostZPt.z)
      {
        mostZPt = cloudRefined->points[i];
      }
      ROS_INFO("aH %.2f", mostZPt.z);
    }
    ROS_INFO("highest Z point %.2f",mostZPt.z);
    //now we find the top and bottom of the led pannel    
    float led2dTop [2];
    led2dTop [0] = mostZPt.z;
    led2dTop [1] = lmFOfx(led2dTop[0]);
    float led2dBot [2];
    led2dBot [0] =(-ledHeight*(1/(sqrt(1+pow(betaHat[1],2)))) + mostZPt.z);
    led2dBot [1] = lmFOfx(led2dBot[0]);

    //double zTop = mostZPt.z-((mostZPt.z-leastZPt.z)*ledTop);
    //double zBot = mostZPt.z-((mostZPt.z-leastZPt.z)*ledBot);
    //now we grab all the partitions between
    pcl::PointCloud<pcl::PointXYZ>::Ptr _ledPts (new pcl::PointCloud<pcl::PointXYZ>);
    ROS_INFO("finding points between %.2e, and %.2e",led2dBot[0],led2dTop[0]);
    for (size_t i=0; i<cloudRefined->size();++i)
    {
      if (cloudRefined->points[i].z>=led2dBot[0] && cloudRefined->points[i].z<led2dTop[0])
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr outlinePts = getOutlinePts(partitions.at(i));
        //now we only push in the most Y, least Y, and least X point form each partition to generate an outline for CV
        
        _ledPts->points.push_back(outlinePts->points[0]);
        _ledPts->points.push_back(outlinePts->points[1]);
        
      }
    }
    ROS_INFO("%i number of ledPts",_ledPts->size());
    return _ledPts;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr getOutlinePts(pcl::PointCloud<pcl::PointXYZ>::Ptr input)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr outlinePts(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ mostYPt =input->points[0];//most left pt
    pcl::PointXYZ leastYPt = input->points[0];//most right pt
    pcl::PointXYZ leastXPt =input->points[0];//closest pt
    
    for (size_t j =0; j<input->size(); ++j)
    {
      if (input->points[j].y>mostYPt.y)
        mostYPt = input->points[j];
      if (input->points[j].y<leastYPt.y)
        leastYPt = input->points[j];
      if (input->points[j].x<leastXPt.x)
        leastXPt = input->points[j]; 
    }
    
    //y delta between the left most(mostY) and middle point(leastX)
    float distL = sqrt(pow(mostYPt.x-leastXPt.x,2)+pow(mostYPt.y-leastXPt.y,2)+pow(mostYPt.z-leastXPt.z,2));
    //y delta between the right most(leastY) and middle point(leastX)
    //float distR = abs(leastYPt.y-leastXPt.y);
    float distR = sqrt(pow(leastYPt.x-leastXPt.x,2)+pow(leastYPt.y-leastXPt.y,2)+pow(leastYPt.z-leastXPt.z,2));
    //float distT = abs(leastYPt.y-mostYPt.y); 
    float distT = sqrt(pow(mostYPt.x-leastYPt.x,2)+pow(mostYPt.y-leastYPt.y,2)+pow(mostYPt.z-leastYPt.z,2));
    if (distT<(distL+distR+.05)&&(distT>(distL+distR-.05)))//if not the corner case, approx a line
    {
      outlinePts->points.push_back(mostYPt);
      outlinePts->points.push_back(leastYPt);//left, then right pt
      return outlinePts;
    }
    //if is corner case 
    if (distR>distL)
    {
      outlinePts->points.push_back(leastXPt);//left then right pt
      outlinePts->points.push_back(leastYPt); 
    }
    else if (distL>distR)
    {
      outlinePts->points.push_back(mostYPt);//left then right pt 
      outlinePts->points.push_back(leastXPt);
    }
    
    //outlinePts->points.push_back(mostYPt); 
    //outlinePts->points.push_back(leastYPt); 
    //outlinePts->points.push_back(leastXPt);
    return outlinePts;
  }
/*
  calcLedPts(pcl::PointCloud<pcl:PointXYZ>::Ptr _ledPts)
  {
    //pcl::PointCloud<pcl::PointXYZ>::Ptr _ledPts(new pcl::PointCloud<pcl::PointXYZ>);
    //first we step through the partitions from the bottom(least z) up to find the bottom of the led pannels
    size_t i = 0;
    for (;i<partitions.size();++i)
    {
      
      if (i==partitions.size()-1)
      {
        ROS_ERROR("the bottom of the LED pannel was never recognized");
        return _ledPts;
      }
      if (isBotOfLed(partitions.at(i),partitions.at(i+1)))
	break;
    }
    ROS_INFO("the bottom of the LED pannel was recognized at partitions %i and %i",i,i+1);
    //ledPts->clear();
    //now we move along the regression line by the hieight of LED pannel and grab the partition below
    //we start at (predicted y, z of the avg XYZ of the upper partition)
    double led2dBot [2];
    led2dBot [0] = cloudRefined->points[i+1].z;
    led2dBot [1] = lmFOfx(cloudRefined->points[i+1].z);
    double led2dTop [2];
    led2dTop [0] = sqrt(pow(ledHeight,2)/(pow(betaHat[1],2)+1))+led2dBot[0];
    led2dTop [1] = lmFOfx(led2dTop[1]);
    //now we add all the points from partitions.at(i+1) to ledPts
    //now we add the points in the partitions between the two 2d pts
    ROS_INFO("Looking for top of LED");
    size_t j=i;
    for (;j<cloudRefined->size();++j)
    {
      if (cloudRefined->points[j].z>led2dTop[0])
        break; 
    }
    ROS_INFO("ledTop at partition: %i ",j);
    ROS_INFO("");
    for (size_t k=i+1; k<=j;++k)
    {
      ROS_INFO("	adding partition %i to LedPts",k);
      for (size_t l=0;j<partitions.at(l)->size();++l)
      {
        _ledPts->points.push_back(partitions.at(k)->points[l]);
      }
      ROS_INFO("	added partition %i to LedPts",k);
    }
    ROS_INFO("");
    return _ledPts;

  }


  bool isBotOfLed//TODO
  (
  pcl::PointCloud<pcl::PointXYZ>::Ptr bot,
  pcl::PointCloud<pcl::PointXYZ>::Ptr top
  )
  {
    //first we say that the num of pts in bot is in some range
    if ((bot->size()>10)||(bot->size()<=0))
      return false;
    
    //then we  say that the y range of bot is like 
    int lowYIdx=0;
    int highYIdx=0;
    for (size_t i=0;i<bot->size();++i)
    {
      if (bot->points[i].y < bot->points[lowYIdx].y)
        lowYIdx = i;
    }
    for (size_t i=0;i<top->size();++i)
    {
      if (bot->points[i].y > bot->points[highYIdx].y)
        highYIdx = i;
    }
    if ((bot->points[highYIdx].y-bot->points[lowYIdx].y)>.05);
      return false; 
    
    //then we say that the num of pts in top is in some other range
    if ((top->size()>20)||(top->size()<5))
      return false;
   
    //then we say that the y range of top is like 
    lowYIdx=0;
    highYIdx=0;
    for (size_t i=0;i<top->size();++i)
    {
      if (top->points[i].y < top->points[lowYIdx].y)
        lowYIdx = i;
    }
    for (size_t i=0;i<top->size();++i)
    {
      if (top->points[i].y > top->points[highYIdx].y)
        highYIdx = i;
    }
    if (((top->points[highYIdx].y-top->points[lowYIdx].y)>.5)||((top->points[highYIdx].y-top->points[lowYIdx].y)<.2));
      return false; 
    //then we say that 
    
    return true;
  }
*/
};//End of class LidarAnalyzer

int main (int argc, char **argv)
{
  ros::init(argc, argv, "scan_the_code_marshall");
  
  LidarAnalyzer instanceOfLidarAnalyzer;
  ros::spin();
  return 0;
}

