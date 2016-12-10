#include "lidarParams.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Critical global constants
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double MAP_SIZE_METERS = 1500;
double ROI_SIZE_METERS = 201;
double VOXEL_SIZE_METERS = 0.30;
int MIN_HITS_FOR_OCCUPANCY = 20; 
int MAX_HITS_IN_CELL = 200; 
int LIDAR_HITS_INCREMENT = 35;
double MAXIMUM_Z_BELOW_LIDAR = 2.25; 
double MAXIMUM_Z_ABOVE_LIDAR = 2.5;
double MAX_ROLL_PITCH_ANGLE_DEG = 2.5;
double LIDAR_VIEW_ANGLE_DEG = 160;
double LIDAR_VIEW_DISTANCE_METERS = 20;
double LIDAR_CONFIDENCE_DISTANCE_METERS = 20;
double LIDAR_MIN_VIEW_DISTANCE_METERS = 5.5;
int MIN_LIDAR_POINTS_FOR_OCCUPANCY = 10;
double MIN_OBJECT_HEIGHT_METERS = 0.15;
double MIN_OBJECT_SEPERATION_DISTANCE = 3.0;
std::vector<std::string> ROIS = {"BuoyField","CoralSurvey","FindBreak","AcousticPinger","Shooter","Scan_The_Code","Gate_1","Gate_2","Gate_3","Dock", "EmptySpace"};
double MIN_GATE_SEPERATION = 30;
double MAX_GATE_SEPERATION = 50;
double MAX_GATE_ERROR_METRIC = 15;
int MIN_HITS_FOR_VOLUME = 31;
int OBJECT_INFLATION_PARAMETER = 2;
double VOXEL_SIZE_Z_METERS = 0.15;
double VOXEL_SIZE_Z_MIN_HITS = 10;
double volumes[5][8] = { 	{1.0,	1.5,	8.0,	10.0,	8.0,	10.0,	2.75,	3.25}, //dock (NOT TESTED!)
        {0.75,	2.25,	3.0,	5.5,	3.0,	5.5,	2.5,	5.5}, //shooter
        {0.1,	0.75,	1.3,	2.25,	1.3,	2.25,	1.95,	2.5}, //scan_the_code
        {-0.6,	0.0,	0.8,	1.8,	0.8,	1.8,	0.8,	1.9}, //totems
        {-1.25,-0.8,	0.125,	1.0,	0.125,	1.0,	0.125,	0.65} }; //buoy
