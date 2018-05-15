#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<nav_msgs/OccupancyGrid.h>
#include<geometry_msgs/Twist.h>
#include<tf/transform_datatypes.h>
#include<tf2/LinearMath/Quaternion.h>
#include<bits/stdc++.h>

using namespace std;

struct state
{
    double x_vel, y_vel, x_pos, y_pos, yaw, yaw_rate;
    ros::Time time;
};

double distance(double x1, double y1, double x2, double y2);
state motion(state temp, double vel_x, double vel_y, double yaw_rate);
double mini_dist(vector<state> trajectory, vector< vector<double> > &distance);
double heading_cost(state current);
double distance_cost(state current, state previous);
double velocity_cost(double vel);
vector< state > calc_trajectory (state init, double velocity, double omega);
vector<double> createDynamicWindow(state currentState);
vector<double> path_find(state current, vector< vector<double> > &distance);
