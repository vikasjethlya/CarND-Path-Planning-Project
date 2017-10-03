/*
 * PathPlanner.cpp
 *
 *  Created on: 01-Oct-2017
 *      Author: vjethlya
 */
#include "PathPlanner.h"
#include "Helper.h"
#include "spline.h"

#include <iostream>
#include <math.h>

using namespace std;
using std::vector;


bool PathPlanner::IsChangeLaneSafe(int keep_lane, double min_left_dist, double min_right_dist, int& lane)
{
	bool lane_change = false;

	if (lane == 0 && min_right_dist >= side_dist && keep_lane == 0 && long_dist > min_long_dist)
	{
		lane += 1;
		lane_change = true;
	}

	else if (lane == 1 && (min_left_dist >= side_dist || min_right_dist >= side_dist) && keep_lane == 0 && long_dist > min_long_dist )
	{
		if (min_left_dist > min_right_dist)
		{
			lane -= 1;
		}
		else
		{
			lane += 1;
		}
		lane_change = true;
	}
	else if (lane == 2 && min_left_dist >= side_dist && keep_lane == 0 && long_dist > min_long_dist)
	{
		lane -= 1;
		lane_change = true;
	}

	return lane_change;
}

void PathPlanner::FollowLane(const vector<double>& map_waypoints_x, const vector<double>& map_waypoints_y, const vector<double>& map_waypoints_s, const vector<double>& previous_path_x, const vector<double>& previous_path_y, double car_x, double car_y, double car_yaw, double car_speed, double car_s, double ref_vel, int lane, vector<double>& next_x_vals, vector<double>& next_y_vals)
{

	int prev_size = previous_path_x.size();
	vector<double> ptsx;
	vector<double> ptsy;

	// reference x and y
	double ref_x = car_x ;
	double ref_y = car_y;
	Helper help;
	double ref_yaw = help.deg2rad(car_yaw);

	// if the previous size is almost empty , use the car as starting reference point
	if(prev_size < 2)
	{
		// Use two points that make path tangent to car

		double prev_car_x = car_x - cos(car_yaw);
		double prev_car_y = car_y - sin(car_yaw);

		ptsx.push_back(prev_car_x);
		ptsx.push_back(car_x);

		ptsy.push_back(prev_car_y);
		ptsy.push_back(car_y);

	}

	// use previous path end points as starting point

	else
	{
		ref_x = previous_path_x[prev_size -1];
		ref_y = previous_path_y[prev_size -1];

		double prev_ref_x = previous_path_x[prev_size -2];
		double prev_ref_y = previous_path_y[prev_size -2];

		ref_yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x );

		car_speed = (help.distance(prev_ref_x, prev_ref_y, ref_x, ref_y) / .02) * 2.237;

		// Use the two points that make the path tangent to the previous's path end point

		ptsx.push_back(prev_ref_x);
		ptsx.push_back(ref_x);

		ptsy.push_back(prev_ref_y);
		ptsy.push_back(ref_y);

	}

	// In Frenet add evenly 30 evenly spaced points ahead of the start points

	vector<double> next_way0 = help.getXY(car_s + 30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_way1 = help.getXY(car_s + 60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_way2 = help.getXY(car_s + 90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

	ptsx.push_back(next_way0[0]);
	ptsx.push_back(next_way1[0]);
	ptsx.push_back(next_way2[0]);

	ptsy.push_back(next_way0[1]);
	ptsy.push_back(next_way1[1]);
	ptsy.push_back(next_way2[1]);

	// Transform and rotation to car coordinate system

	for(unsigned int i = 0 ; i < ptsx.size(); i++)
	{
		double shift_x = ptsx[i] - ref_x;
		double shift_y = ptsy[i] - ref_y;

		ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
		ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));

	}

	// create a spline
	tk::spline s;

	// set x and y points to spline
	s.set_points(ptsx, ptsy);

	// start with all of the previous path points from the last time

	for(unsigned int i = 0 ; i < previous_path_x.size() ; i++)
	{
		next_x_vals.push_back(previous_path_x[i]);
		next_y_vals.push_back(previous_path_y[i]);

	}

	// calculate how to break up spline points so that we travel to our desired velocity

	double target_x = 30.0;
	double target_y = s(target_x);
	double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));

	double x_add_on = 0;

	// fill up the rest of our plth planner points after filling it with previous points, here we always display
	// 50 points

	for(unsigned int i = 1  ; i <= 50 - previous_path_x.size() ; i++)
	{
		if (car_speed < ref_vel)
		{
			car_speed += .224;
		}
		else if (car_speed > ref_vel)
		{
			car_speed -= .224;
		}

		double N = (target_dist/(0.02 * car_speed/2.24));
		double x_point = x_add_on + (target_x)/N ;
		double y_point = s(x_point);

		x_add_on = x_point;

		double x_ref = x_point;
		double y_ref = y_point;

		//rotate back to normal after rotating it earlier

		x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
		y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

		x_point += ref_x;
		y_point += ref_y;

		next_x_vals.push_back(x_point);
		next_y_vals.push_back(y_point);

	}
}

void PathPlanner::ChangeLane(bool too_close, double min_left_dist, double min_right_dist, int &lane, int& keep_lane)
{
	bool changed_lane = IsChangeLaneSafe(keep_lane, min_left_dist, min_right_dist, lane);

		if (changed_lane) {
			keep_lane = 40;
		}
}

void PathPlanner::GetCurrentVelocity(int lane, double ego_car_s, double ego_car_d, const vector<vector<double>>& sensor_fusion, int& prev_size, double& ref_vel, double& min_left_dist, double& min_right_dist, bool& too_close, double& closest)
{
	int sensor_fusion_size = sensor_fusion.size();
	double lane_size = 4 * lane;
    long_dist = 999;
	Helper help;

	for (int i = 0; i < sensor_fusion_size; i++)
	{
		double car_id = sensor_fusion[i][0];
		double car_x = sensor_fusion[i][1];
		double car_y = sensor_fusion[i][2];
		double car_vx = sensor_fusion[i][3];
		double car_vy = sensor_fusion[i][4];
		double car_s = sensor_fusion[i][5];
		double car_d = sensor_fusion[i][6];
		double distance = help.distance(car_s, car_d, ego_car_s, ego_car_d);

		if ((ego_car_d - car_d) < -1.0 && distance < min_right_dist)
		{
			min_right_dist = distance;
		}
		else if ((ego_car_d - car_d) > 1.0 && distance < min_left_dist)
		{
			min_left_dist = distance;
		}

        double check_speed = sqrt(car_vx * car_vx + car_vy * car_vy);
        
        double check_car_s = car_s;
        
        check_car_s += ((double)prev_size * .02 * check_speed);
        
		if (car_d < (2 + lane_size + 2) && car_d > (2+ lane_size - 2))
		{
			if (check_car_s > ego_car_s)
			{
				closest = check_car_s - ego_car_s;

				if (closest < closest_dist + 20)
				{
					too_close = true;
				}

				if (closest < closest_dist + 10)
				{
					ref_vel = check_speed * 2.237;

					if (closest < closest_dist)
					{
						ref_vel -= 5;
					}
				}
			}
		}
        if (check_car_s < ego_car_s && (ego_car_s - check_car_s) < long_dist)
        {
            long_dist = ego_car_s - check_car_s;
            cout << " S distance :" << long_dist << endl;
        }
	}
}
