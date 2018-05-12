
struct state
{
    double x_vel, y_vel, x_pos, y_pos, yaw, yaw_rate;
};

state latest;
vector< vector<double> > map;

void mapcallback(nav_msgs::Odometry::ConstPtr &vel)
{

}

void velcallback(nav_msgs::OccupancyGrid::ConstPtr &map_o)
{

}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"dwa");
    ros::NodeHandle n;

    ros::Subscriber map_s = n.subscribe("/map",10,&mapcallback);
    ros::Subscriber vel_s = n.subscribe("/odometery/filtered",10,&velcallback);
    ros::Publisher out_p = n.publish<nav_msgs::Odometry>("/cmd_vel",10);

    while(ros::ok())
    {
        ros::spinOnce();

    }
    return 0;
}