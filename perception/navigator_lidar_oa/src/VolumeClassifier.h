////////////////////////////////////////////////////////////
//
// Volume Classifier
//
////////////////////////////////////////////////////////////
#ifndef VOLUMECLASSIFIER_H
#define VOLUMECLASSIFIER_H

#include <vector>
#include <string>
#include "ConnectedComponents.h"
#include "navigator_msgs/PerceptionObject.h"
#include <algorithm>
#include "lidarParams.h"

void VolumeClassifier(objectMessage &object)
{
	//Seperate the object size based on all measurements and just including close measurements
	//auto h = object.maxHeightFromLidar;
	auto dxAll = object.scaleAll.x;
	auto dyAll = object.scaleAll.y;
	auto dzAll = object.scaleAll.z;

    auto dxClose = object.scaleClose.x;
    auto dyClose = object.scaleClose.y;
    auto dzClose = object.scaleClose.z;

	//Skip classification if the object is locked, set as a start_gate, or if it isn't real
	if (object.locked || object.name == navigator_msgs::PerceptionObject::START_GATE_BUOY || !object.real) {
		return;
	}

	//Possible classifications
    std::vector<std::string> classifications = {"dock","shooter","scan_the_code","totem","buoy","large","medium","small"};

    //Update object confidnce using all lidar data
    if (dzAll > 2.75) {
        ++object.confidence[5];
    } else if (dzAll > 0.9) {
        ++object.confidence[6];
    } else {
        ++object.confidence[7];
    }


    //Update object confidence using only close lidar data   
    //std::vector<std::tuple<int,int>> options = { make_tuple(object.confidence[5],ii),make_tuple(object.confidence[6],6),make_tuple(object.confi);
    for (auto ii = 0; ii < classifications.size()-3; ++ii) {
       	//if ( h >= volumes[ii][0] && h <= volumes[ii][1]	&& ( (x >= volumes[ii][2] && x <= volumes[ii][3]) ||  (y >= volumes[ii][4] && y <= volumes[ii][5]) ) && z >= volumes[ii][6]	&& z <= volumes[ii][7] )  {
        if ( ((dxClose >= volumes[ii][2] && dxClose <= volumes[ii][3]) ||  (dyClose >= volumes[ii][4] && dyClose <= volumes[ii][5])) && dzClose >= volumes[ii][6] && dzClose <= volumes[ii][7]) {
           	if (object.confidence[ii] < size_t(-1) ) { ++object.confidence[ii];	}		
        }
		//if (object.confidence[ii] >= MIN_HITS_FOR_VOLUME ) { 
        //    options.push_back(std::make_tuple(object.confidence[ii],ii));
        //}
    }

    //Confidence has to meet minimum threshold before being selected
    object.name = "unknown";
    object.bestConfidence = 0;
    auto d = std::distance(object.confidence.begin(),std::max_element(object.confidence.begin(),object.confidence.begin()+4));
    if (object.confidence[d] > MIN_HITS_FOR_VOLUME) {
            object.name = classifications[d];
            object.bestConfidence = (double)object.confidence[d] / std::accumulate(object.confidence.begin(),object.confidence.begin()+4,0.0)*255.0;
    } else {
        auto d2 = std::distance(object.confidence.begin(),std::max_element(object.confidence.begin()+5,object.confidence.end()));
        object.name = classifications[d2];
    }
    
        /*if (ii < names.size()-1 && object.confidence[ii] >= MIN_HITS_FOR_VOLUME && object.confidence[ii] > object.confidence[ii+1]/2 ) { 
            object.name = names[ii];
            object.bestConfidence = (double)object.confidence[ii]/std::accumulate(object.confidence.begin(),object.confidence.end(),0.0)*255;
            break;
       } else if (ii == names.size()-1 && object.confidence[ii] >= MIN_HITS_FOR_VOLUME) {
            object.name = names[ii];
            object.bestConfidence = (double)object.confidence[ii]/std::accumulate(object.confidence.begin(),object.confidence.end(),0.0)*255;
            break;       	
       }*/
}
#endif
