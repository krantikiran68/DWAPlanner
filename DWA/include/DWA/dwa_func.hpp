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

struct point
{
    int x,y;

    point(int a, int b)
    {
        x=a;
        y=b;
    }

    bool operator==(const point& a) const
    {
        return (x == a.x && y == a.y);
    }
};

double distance(double x1, double y1, double x2, double y2);

class dwa
{       
        double max_speed_f;
        double max_speed_b;
        double max_yaw_rate;
        double max_accel;
        double max_yaw_acc;
        double vel_reso;
        double yaw_reso;
        double cycle_time;
        double dt;
        double cons_rad;
        double min_obj_dist;
        int free_thresh;
        nav_msgs::OccupancyGrid cur_map_f;
        double map_yaw;
        vector<double> map_ori;

        state motion(state temp, double vel_x, double vel_y, double yaw_rate);
        bool is_valid(int x,int y);
        double traj_dist(state curent);
        double mini_dist(vector<state> trajectory);
        double heading_cost(state current);
        double distance_cost(state current, state previous);
        double velocity_cost(double vel);
        vector< state > calc_trajectory (state init, double velocity, double omega);
        vector<double> createDynamicWindow(state currentState);
        
    public:
        double x_goal;
        double y_goal;
        double goal_res;

        vector<double> path_find(state current,ros::NodeHandle n);
        void param_initialize(ros::NodeHandle n);
};