#include<DWA/New.hpp>

using namespace std;

extern double max_speed;
extern double min_speed;
extern double max_omega;
extern double max_accel;
extern double max_domega;
extern double vel_reso;
extern double omega_reso;
extern double predict_time;
extern double x_goal;
extern double y_goal;
extern double dt;
extern double goal_res;
extern double cons_rad;

state latest;
bool flag=false;

void velcallback(const nav_msgs::Odometry::ConstPtr& vel)
{
    latest.time  = ros::Time(vel->header.stamp.sec);

    latest.x_pos = vel->pose.pose.position.x;
    latest.y_pos = vel->pose.pose.position.y;
    latest.x_vel = vel->twist.twist.linear.x;
    latest.y_vel = vel->twist.twist.linear.y;
    latest.yaw_rate = vel->twist.twist.angular.z;

    tf::Quaternion q;
    tf::quaternionMsgToTF(vel->pose.pose.orientation,q);
    latest.yaw = getYaw(q);

    flag = true;
}

void mapcallback(const nav_msgs::OccupancyGrid::ConstPtr& map_o)
{
    return;
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"dwa");
    ros::NodeHandle n;

    ros::Subscriber map_s = n.subscribe("/map",10,&mapcallback);
    ros::Subscriber vel_s = n.subscribe("/odometery/filtered",10,&velcallback);
    ros::Publisher  out_p = n.advertise<geometry_msgs::Twist>("/cmd_vel",10);

    while(ros::ok())
    {
        ros::spinOnce();
        if(!flag)
            continue;
        flag = false;
        
        if(distance(latest.x_pos,x_goal,latest.y_pos,y_goal) < goal_res)
            break;
        
        vector<double> vel_ome = path_find(latest);

        geometry_msgs::Twist vel_fi;
        vel_fi.linear.x = vel_ome[0];
        vel_fi.angular.z = vel_ome[1];

        out_p.publish(vel_fi);
    }
    return 0;
}