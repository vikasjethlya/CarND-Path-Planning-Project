#ifndef Helper_H
#define Helper_H

#include <iostream>
#include<vector>

using namespace std;

using std::vector;

class Helper
{
	public:

		bool debug_print = false; //set to false to disable consle print

		double pi();
		double deg2rad(double x);
		double rad2deg(double x);
		string hasData(string s);
		double distance(double x1, double y1, double x2, double y2);
		int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);
		int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
		vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
		vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

		//Prints a text
		void DebugInfo(const string& text);

		//Prints Ego's data car
		void DebugInfoEgoCarData(double x, double y, double yaw, double speed, double s, double d, double ref_vel);

		//Prints data for the other vehicles in the road
		void DebugInfoVehicleData(double ego_car_s, double ego_car_d, const vector<vector<double>>& sensor_fusion);

		//Prints data for the closest obstacle
		void DebugInfoObstacleData(double min_left_dist, double min_right_dist, double closest, double too_close);

		//data for the current lane
		void DebugInfoLaneData(int lane);

};

#endif
