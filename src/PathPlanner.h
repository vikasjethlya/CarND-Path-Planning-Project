/*
 * PathPlanner.h
 *
 *  Created on: 30-Sep-2017
 *      Author: vjethlya
 */

#ifndef SUBPROJECTS__PATH_PLANNING_PATHPLANNER_H_
#define SUBPROJECTS__PATH_PLANNING_PATHPLANNER_H_


#include <iostream>
#include <vector>

using namespace std;

using std::vector;

class PathPlanner
{
	private :

		double lane_width = 4.0;
		double speed_limit = 49.5;
		double side_dist = 20;
		double closest_dist = 20;
        double min_long_dist = 20.0;
	public:
        double long_dist = 999;

		void FollowLane(const vector<double>& map_waypoints_x, const vector<double>& map_waypoints_y,
				const vector<double>& map_waypoints_s, const vector<double>& previous_path_x,
				const vector<double>& previous_path_y, double car_x, double car_y, double car_yaw,
				double car_speed, double car_s, double ref_vel, int lane, vector<double>& next_x_vals,
				vector<double>& next_y_vals);

		bool IsChangeLaneSafe(int keep_lane, double min_left_dist, double min_right_dist, int& lane);

		void ChangeLane(bool too_close, double min_left_dist, double min_right_dist, int &lane, int& keep_lane);

		void GetCurrentVelocity(int lane, double ego_car_s, double ego_car_d,
				const vector<vector<double>>& sensor_fusion, int& prev_size, double& ref_vel,
				double& min_left_dist, double& min_right_dist, bool& too_close, double& closest);
};

#endif /* SUBPROJECTS__PATH_PLANNING_PATHPLANNER_H_ */
