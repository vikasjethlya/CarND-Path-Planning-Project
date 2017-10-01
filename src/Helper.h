#ifndef Helper_H
#define Helper_H

#include <iostream>
#include<vector>

using namespace std;

using std::vector;

class Helper
{
	public:
		double pi();
		double deg2rad(double x);
		double rad2deg(double x);
		string hasData(string s);
		double distance(double x1, double y1, double x2, double y2);
		int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);
		int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
		vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
		vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);
};

#endif
