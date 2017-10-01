/*
 * Helper.cpp
 *
 *  Created on: 01-Oct-2017
 *      Author: vjethlya
 */


#include "Helper.h"
#include <iostream>
#include <math.h>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

using std::vector;


double Helper::pi()
{
	return M_PI;
}

// For converting back and forth between radians and degrees.

double Helper::deg2rad(double x)
{
	return x * pi() / 180;
}

double Helper::rad2deg(double x)
{
	return x * 180 / pi();
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.

string Helper::hasData(string s)
{
	 auto found_null = s.find("null");
	  auto b1 = s.find_first_of("[");
	  auto b2 = s.find_first_of("}");
	  if (found_null != string::npos) {
	    return "";
	  } else if (b1 != string::npos && b2 != string::npos) {
	    return s.substr(b1, b2 - b1 + 2);
	  }
	  return "";
}

double Helper::distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int Helper::ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{
	double closestLen = 100000; //large number
		int closestWaypoint = 0;

		for(int i = 0; i < maps_x.size(); i++)
		{
			double map_x = maps_x[i];
			double map_y = maps_y[i];
			double dist = distance(x,y,map_x,map_y);
			if(dist < closestLen)
			{
				closestLen = dist;
				closestWaypoint = i;
			}

		}
		return closestWaypoint;
}

int Helper::NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

		double map_x = maps_x[closestWaypoint];
		double map_y = maps_y[closestWaypoint];

		double heading = atan2( (map_y-y),(map_x-x) );

		double angle = abs(theta-heading);

		if(angle > pi()/4)
		{
			closestWaypoint++;
		}

		return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates

vector<double> Helper::getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

		int prev_wp;
		prev_wp = next_wp-1;
		if(next_wp == 0)
		{
			prev_wp  = maps_x.size()-1;
		}

		double n_x = maps_x[next_wp]-maps_x[prev_wp];
		double n_y = maps_y[next_wp]-maps_y[prev_wp];
		double x_x = x - maps_x[prev_wp];
		double x_y = y - maps_y[prev_wp];

		// find the projection of x onto n
		double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
		double proj_x = proj_norm*n_x;
		double proj_y = proj_norm*n_y;

		double frenet_d = distance(x_x,x_y,proj_x,proj_y);

		//see if d value is positive or negative by comparing it to a center point

		double center_x = 1000-maps_x[prev_wp];
		double center_y = 2000-maps_y[prev_wp];
		double centerToPos = distance(center_x,center_y,x_x,x_y);
		double centerToRef = distance(center_x,center_y,proj_x,proj_y);

		if(centerToPos <= centerToRef)
		{
			frenet_d *= -1;
		}

		// calculate s value
		double frenet_s = 0;
		for(int i = 0; i < prev_wp; i++)
		{
			frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
		}

		frenet_s += distance(0,0,proj_x,proj_y);

		return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> Helper::getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

		while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
		{
			prev_wp++;
		}

		int wp2 = (prev_wp+1)%maps_x.size();

		double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
		// the x,y,s along the segment
		double seg_s = (s-maps_s[prev_wp]);

		double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
		double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

		double perp_heading = heading-pi()/2;

		double x = seg_x + d*cos(perp_heading);
		double y = seg_y + d*sin(perp_heading);

		return {x,y};
}

/**
* Prints a text
*/
void Helper::DebugInfo(const string& text)
{
	if (debug_print)
	{
		cout << text << endl;
	}
}

/**
* Prints Ego's data car
*/
void Helper::DebugInfoEgoCarData(double x, double y, double yaw, double speed, double s, double d, double ref_vel)
{
	if (debug_print)
	{
		cout << "\tx: " << x << "\ty: " << y
			<< "\tyaw: " << yaw << "\tspeed: " << speed
			<< "\ts: " << s << "\td: " << d
			<< endl;
		cout << "Ref Speed: " << ref_vel << "mph" << endl;
	}
}

/**
* Prints data for the other vehicles in the road
*/
void Helper::DebugInfoVehicleData(double ego_car_s, double ego_car_d, const vector<vector<double>>& sensor_fusion)
{
	if (debug_print)
	{
		for (int i = 0; i < sensor_fusion.size(); i++)
		{
			double id = sensor_fusion[i][0];
			double x = sensor_fusion[i][1];
			double y = sensor_fusion[i][2];
			double vx = sensor_fusion[i][3];
			double vy = sensor_fusion[i][4];
			double s = sensor_fusion[i][5];
			double d = sensor_fusion[i][6];
			double distance = sqrt((ego_car_s - s) * (ego_car_s - s) + (ego_car_d - d) * (ego_car_d - d));
			cout << "Car ID: " << id
				<< "\tx: " << x << "\ty: " << y
				<< "\tvx: " << vx << "\tvy: " << vy
				<< "\ts: " << s << "\td: " << d
				<< "\tdistance: " << distance
				<< endl;
		}

	}
}

/**
* Prints data for the closest obstacle
*/
void Helper::DebugInfoObstacleData(double min_left_dist, double min_right_dist, double closest, double too_close)
{
	if (debug_print)
	{
		cout << "------- OBSTACLES -------" << endl;
		cout << "Left Distance: " << min_left_dist << "m" << endl;
		cout << "Current lane Distance: " << closest << "m";
		cout<<"\t\t Too close to collide : ";
		if(too_close)
		{
			cout << "Yes" << endl;
		}
		else
		{
			cout << "No" << endl;
		}
		cout << "Right Distance: " << min_right_dist << "m" << endl;
	}
}

/**
* Prints data for the current lane
*/
void Helper::DebugInfoLaneData(int lane)
{
	if (debug_print)
	{
		cout << "------- LANE INFO -------" << endl;
		cout << "Current lane: " ;
		switch(lane)
		{
			case 0:
				cout<<"Left";
				break;
			case 1:
				cout << "Middle";
				break;
			case 2:
				cout<< "Right";
				break;
			default:
				cout<< "Invalid Lane";
		}
		cout<<endl;
	}
}
