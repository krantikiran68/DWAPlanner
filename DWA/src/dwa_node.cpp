#include<DWA/dwa_func.hpp>

using namespace std;

nav_msgs::OccupancyGrid cur_map;
state latest;
dwa bot;
bool flag=false;
bool flag1=false;
bool flagm=false;

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
    cur_map = *map_o;
    flag1=true;
}

void goalcallback(const geometry_msgs::Pose::ConstPtr& goal_c)
{
    bot.x_goal=goal_c->position.x;
    bot.y_goal=goal_c->position.y;
    flagm=true;
}

void dwa::param_initialize(ros::NodeHandle n)
{
    if (!(n.getParam("max_speed_f", max_speed_f)))
        max_speed_f= 1;
    if (!(n.getParam("max_speed_b", max_speed_b)))
        max_speed_b= -0.5;
    if (!(n.getParam("max_accel", max_accel)))
        max_accel= 0.3;
    if (!(n.getParam("max_yaw_rate", max_yaw_rate)))
        max_yaw_rate= 40.0 * M_PI / 180.0  ;
    if (!(n.getParam("max_yaw_acc", max_yaw_acc)))
        max_yaw_acc= 20.0 * M_PI / 180.0 ;
    if (!(n.getParam("vel_reso", vel_reso)))
        vel_reso=0.05;
    if (!(n.getParam("yaw_reso", yaw_reso)))
        yaw_reso=0.1 * M_PI / 180.0;
    if (!(n.getParam("cycle_time", cycle_time)))
        cycle_time=6;
    if (!(n.getParam("dt", dt)))
        dt=0.2;
    if (!(n.getParam("goal_res", goal_res)))
        goal_res=0.1;   
    if (!(n.getParam("free_thresh", free_thresh)))
        free_thresh=19;
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"dwa");
    ros::NodeHandle n;

    ros::Subscriber map_s = n.subscribe("/map",10,&mapcallback);
    ros::Subscriber vel_s = n.subscribe("/odometery/filtered",10,&velcallback);
    ros::Subscriber goal = n.subscribe("/goal",10,&goalcallback);
    ros::Publisher  out_p = n.advertise<geometry_msgs::Twist>("/cmd_vel",10);

    bot.param_initialize(n);
    while(ros::ok())
    {
        ros::spinOnce();
        if(!flag || !flag1 || !flagm)
            continue;
        flag = false;
        
        if(distance(latest.x_pos,bot.x_goal,latest.y_pos,bot.y_goal) < bot.goal_res)
        {
            ROS_INFO("GOAL REACHED");
            break;
        }
        
        vector<double> vel_ome = bot.path_find(latest,n);

        geometry_msgs::Twist vel_fi;
        vel_fi.linear.x = vel_ome[0];
        vel_fi.angular.z = vel_ome[1];

        out_p.publish(vel_fi);
    }
    return 0;
}