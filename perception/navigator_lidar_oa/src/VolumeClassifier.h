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
	//Seperate variables we care about
	auto h = object.maxHeightFromLidar;
	auto x = object.scale.x;
	auto y = object.scale.y;
	auto z = object.scale.z;

	//Skip classification if the object is locked, set as a start_gate, or if it isn't real
	if (object.locked || object.name == navigator_msgs::PerceptionObject::START_GATE_BUOY || !object.real) {
		return;
	}

	//Possible classifications
	std::vector<std::string> names = {"dock","shooter","scan_the_code","totem","buoy"};

	//Volume boundaries - low h, high h, low x, high x, low y, high y, low z, high z
	double volumes[5][8] = { 	{1.0,	1.5,	8.0,	10.0,	8.0,	10.0,	2.75,	3.25}, //dock (NOT TESTED!)
					{0.75,	2.25,	3.0,	5.5,	3.0,	5.5,	2.5,	5.5}, //shooter
					{0.1,	0.75,	1.3,	2.25,	1.3,	2.25,	1.7,	2.5}, //scan_the_code
					{-0.6,	0.0,	0.8,	1.8,	0.8,	1.8,	0.8,	1.8}, //totems
					{-1.25,-0.8,	0.125,	1.0,	0.125,	1.0,	0.125,	0.65} }; //buoy

	//Update confidence for each type of object
    std::vector<std::tuple<int,int>> options;
    for (auto ii = 0; ii < names.size(); ++ii) {
       	if ( h >= volumes[ii][0] && h <= volumes[ii][1]	&& ( (x >= volumes[ii][2] && x <= volumes[ii][3]) ||  (y >= volumes[ii][4] && y <= volumes[ii][5]) ) && z >= volumes[ii][6]	&& z <= volumes[ii][7] )  {
           	if (object.confidence[ii] < size_t(-1) ) { ++object.confidence[ii];	}		
		}
		if (object.confidence[ii] >= MIN_HITS_FOR_VOLUME ) { 
            options.push_back(std::make_tuple(object.confidence[ii],ii));
        }
    }

    //Confidence has to meet minimum threshold before being selected
    object.name = "unknown";
    object.bestConfidence = 0;
    for (auto ii = 0; ii < names.size(); ++ii) {
        if (ii < names.size()-1 && object.confidence[ii] >= MIN_HITS_FOR_VOLUME && object.confidence[ii] > object.confidence[ii+1]/2 ) { 
            object.name = names[ii];
            object.bestConfidence = (double)object.confidence[ii]/std::accumulate(object.confidence.begin(),object.confidence.end(),0.0)*255;
            break;
       } else if (ii == names.size()-1 && object.confidence[ii] >= MIN_HITS_FOR_VOLUME) {
            object.name = names[ii];
            object.bestConfidence = (double)object.confidence[ii]/std::accumulate(object.confidence.begin(),object.confidence.end(),0.0)*255;
            break;       	
       } 
    }
}
#endif
