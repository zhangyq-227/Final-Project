#pragma once
#include <iostream>
#include <math.h>
#include <vector>
using namespace std;
#define pi acos(-1)
 
//waypoint
typedef struct waypoint {
	int ID;
	double x, y, yaw;
}waypoint;
 
//vehicleState
typedef struct vehicleState {
	double x, y, yaw, v, kesi;
}vehicleState;
 
//control volume
typedef struct U {
	double v;
	double kesi;//Front wheel deviation angle
}U;

double cal_K(vector<waypoint> waypoints, int index);//curvature
