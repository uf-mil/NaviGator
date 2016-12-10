////////////////////////////////////////////////////////////
//
// Connected components for ROS
//
////////////////////////////////////////////////////////////
#ifndef CONNECTEDCOMPONENTS_H
#define CONNECTEDCOMPONENTS_H

#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <map>
#include <set>
#include <unordered_map>
#include "OccupancyGrid.h"
#include "lidarParams.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct objectMessage
{
	void insertPersist(const std::deque<LidarBeam> &strikes) {
		persist.insert(persist.end(),strikes.begin(),strikes.end());
	}
	void insertFrame(const std::vector<LidarBeam> &strikes) {
		frame.insert(frame.end(),strikes.begin(),strikes.end());	
	}
	bool dimensions() {
		//
		std::multiset<double> xAll,yAll,zAll,xClose,yClose,zClose;
		std::unordered_map<int,unsigned> map;
		size_t cnt = 0;
		for (auto p = persist.begin(); p != persist.end(); ++p, ++cnt) {
			//if (true) {
			if (fabs(p->boatAnglesDot.roll) <= 0.004 && fabs(p->boatAnglesDot.pitch) <= 0.009 && fabs(p->boatAnglesDot.yaw) <= 0.008 ) {
				xAll.insert(p->x); yAll.insert(p->y); zAll.insert(p->z);
				if (p->d <= LIDAR_CONFIDENCE_DISTANCE_METERS) {
					xClose.insert(p->x); yClose.insert(p->y); zClose.insert(p->z);	
					//++map[floor(p->z/VOXEL_SIZE_Z_METERS)];
				}
			}
		}

		//REMOVE OUTLIERS		
/*		auto removed = 0;
		for (auto it = z.begin(); it != z.end(); ) {
			if (map[floor(*it/VOXEL_SIZE_Z_METERS)] < VOXEL_SIZE_Z_MIN_HITS) {
				it = z.erase(it); ++removed;
			} else {
				++it;
			}
		}*/

		if (xAll.size() > 1 && yAll.size() > 1 && zAll.size() > 1) {
			scaleAll.x = *(--xAll.end()) - *xAll.begin(); position.x = *xAll.begin() + scaleAll.x/2;
			scaleAll.y = *(--yAll.end()) - *yAll.begin(); position.y = *yAll.begin() + scaleAll.y/2;
			scaleAll.z = *(--zAll.end()) - *zAll.begin(); position.z = *zAll.begin() + scaleAll.z/2;
			scale = scaleAll;
		}

		if (xClose.size() > 1 && yClose.size() > 1 && zClose.size() > 1) {
			scaleClose.x = *(--xClose.end()) - *xClose.begin(); position.x = *xClose.begin() + scaleClose.x/2;
			scaleClose.y = *(--yClose.end()) - *yClose.begin(); position.y = *yClose.begin() + scaleClose.y/2;
			scaleClose.z = *(--zClose.end()) - *zClose.begin(); position.z = *zClose.begin() + scaleClose.z/2;
			scale = scaleClose;
		}

		ROS_INFO_STREAM("LIDAR | All   DIMENSIONS -> " << scaleAll.x << "\t" << scaleAll.y << "\t" << scaleAll.z);
		ROS_INFO_STREAM("LIDAR | Close DIMENSIONS -> " << scaleClose.x << "\t" << scaleClose.y << "\t" << scaleClose.z);
		return true;
	}

	geometry_msgs::Point position;
	geometry_msgs::Vector3 scale,scaleAll,scaleClose;
	std::vector<LidarBeam> persist;
	std::vector<LidarBeam> frame;
	std_msgs::ColorRGBA color;
	int id = -1;
	std::string name = "unknown";
	uint32_t pclInliers = 0;
	geometry_msgs::Vector3 normal;
	float minHeightFromLidar,maxHeightFromLidar;
	bool current = false;
	bool locked = false;
	bool real = true;
	ros::Time age;
    std::array<size_t,8> confidence{ {0,0,0,0,0,0,0,0} };
    uint8_t bestConfidence = 0;
};


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct RCLabel
{
	RCLabel(int r, int c, int l) : row(r), col(c), label(l) {}
	int row,col,label;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector< std::vector<int> > ConnectedComponents(OccupancyGrid &ogrid, std::vector<objectMessage> &objects)
{
	//std::cout << "STARTING CONNECTED COMPONENTS" << std::endl;

	std::vector<std::vector<int> > cc(ogrid.ROI_SIZE, std::vector<int>(ogrid.ROI_SIZE,0));
	const int NEIGHBORS = 4;
	int row_neighbor[NEIGHBORS] = {0,-1,1,-1};
	int col_neighbor[NEIGHBORS] = {-1,0,-1,-1};

	int label = 0;

	std::map<int,int> labelMap;

	for (int row = 1; row < ogrid.ROI_SIZE-1; ++row) {
		for (int col = 1; col < ogrid.ROI_SIZE-1; ++col) {
			//Is this a good pixel?
			if (ogrid.ogridBinary[row][col]) {
				//Check neighbors to the right and down
				std::vector<RCLabel> neighbors;
				for (int kk = 0; kk < NEIGHBORS; ++kk) {
					int nrow = row_neighbor[kk]+row;
					int ncol = col_neighbor[kk]+col;					
					//if (nrow >= 0 && ncol >= 0) {
						if ( cc[nrow][ncol] ) {
							neighbors.push_back(RCLabel(nrow,ncol,cc[nrow][ncol]));
						}
					//}
				}
				
				//Was this the first neighbor?
				if (neighbors.size() == 0) {
					++label;
					//std::cout << "New label at " << row << "," << col << " with label " << label << std::endl;
					cc[row][col] = label;
					labelMap[label] = label;
				} else {
					RCLabel min_label = *min_element(neighbors.begin(),neighbors.end(),[](RCLabel &left, RCLabel &right) { return left.label < right.label; });
					//int max_label = *max_element(neighbors.begin(),neighbors.end());
					//std::cout << "Choosing minimum label of " << min_label << " - " << max_label << " at " << row << "," << col << std::endl;
					cc[row][col] = min_label.label;
					//4 way
					////if ( labelMap.count(max_label) == 0 || labelMap[max_label] > min_label) { 
					for (auto ii : neighbors) {
						if (labelMap[ii.label] > min_label.label) {
							labelMap[ii.label] = min_label.label; 
						}
					}
				}
			}
		}
	}

	//Organize labels
	std::set<int> ids;
	for (auto ii : labelMap)  {
		int index = ii.first, match = ii.second;
		//std::cout << "old label " << ii.first << " becomes " << ii.second << std::endl;
		while (true) {
			if (labelMap[match] != match && labelMap[match] > 0) {
				labelMap[index] = labelMap[match]; match = labelMap[match];
			} else {
				break;
			}
		}
		ids.insert(match);
		//std::cout << "new label " << index << " becomes " << match << std::endl;
	}

	//Pass 2
	std::map<int,objectMessage> mapObjects;
	for (int row = 0; row < ogrid.ROI_SIZE; ++row) {
		for (int col = 0; col < ogrid.ROI_SIZE; ++col) {
			if (cc[row][col]) { 
				cc[row][col] = labelMap[cc[row][col]]; 
				//mapObjects[cc[row][col]].update(row,col,ogrid.ogrid[row + ogrid.boatRow - ogrid.ROI_SIZE/2][col + ogrid.boatCol - ogrid.ROI_SIZE/2]);
				int r = row + ogrid.boatRow - ogrid.ROI_SIZE/2, c = col + ogrid.boatCol - ogrid.ROI_SIZE/2;
				mapObjects[cc[row][col]].insertPersist(ogrid.pointCloudTable[r*ogrid.GRID_SIZE+c].q);
				mapObjects[cc[row][col]].insertFrame(ogrid.pointCloudTable_Uno[r*ogrid.GRID_SIZE+c]);
			}
			//std::cout << cc[ii][jj] << " ";
		}
		//std::cout << std::endl;
	}

	//Re-organize obstacles, how many connections to count as an obstacle?
	objects.clear();
	for (auto &ii : mapObjects)  {
		if ( !ii.second.dimensions() ) { continue; }
		bool isNewObject = true;
		objectMessage obj = ii.second;
		obj.age = ros::Time::now();
		//obj.maxHeightFromLidar = (ii.second.scale.z/2+ii.second.position.z)-ogrid.lidarPos.z;
		//Is this object really part of another one?
		/*
		for (auto &jj : objects) {
			auto distance = sqrt(pow(obj.position.x-jj.position.x,2) + pow(obj.position.y-jj.position.y,2)  );
			if (distance <= MIN_OBJECT_SEPERATION_DISTANCE) {
				ROS_INFO_STREAM("LIDAR | Merging object together: " << jj.position.x << "," << jj.position.y << "," << jj.position.z << " vs " << obj.position.x << "," << obj.position.y << "," << obj.position.z );
				ROS_INFO_STREAM("LIDAR | Merging object together: " << jj.strikesPersist.size() << "," << jj.strikesFrame.size() << " vs " << obj.strikesPersist.size() << "," << obj.strikesFrame.size() );
				//ros::Duration(5).sleep();
				jj.position.x = (obj.position.x+jj.position.x)/2;
				jj.position.y = (obj.position.y+jj.position.y)/2;
				jj.position.z = (obj.position.z+jj.position.z)/2;
				jj.strikesPersist.insert(jj.strikesPersist.end(),obj.strikesPersist.begin(),obj.strikesPersist.end() );
				jj.strikesFrame.insert(jj.strikesFrame.end(),obj.strikesFrame.begin(),obj.strikesFrame.end() );
				jj.intensityPersist.insert(jj.intensityPersist.end(),obj.intensityPersist.begin(),obj.intensityPersist.end() );
				jj.intensityFrame.insert(jj.intensityFrame.end(),obj.intensityFrame.begin(),obj.intensityFrame.end() );
				jj.minHeightFromLidar = std::min(jj.minHeightFromLidar,obj.minHeightFromLidar);
				jj.maxHeightFromLidar = std::max(jj.maxHeightFromLidar,obj.maxHeightFromLidar);
				isNewObject = false;
				//BOOST_ASSERT_MSG(false, "yolo");
				break;
			}
		}*/

		if (obj.scaleAll.z >= MIN_OBJECT_HEIGHT_METERS && isNewObject) {
			ROS_INFO_STREAM("LIDAR | CC New object added!");
			objects.push_back(obj);
		} else {
			ROS_INFO_STREAM("LIDAR | CC Object rejected...");
		}
	}
	return cc;
}
#endif
