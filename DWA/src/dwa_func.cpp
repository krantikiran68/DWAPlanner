#include<DWA/dwa_func.hpp> 

using namespace std;

extern nav_msgs::OccupancyGrid cur_map; 

double distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}

state dwa::motion(state temp, double vel_x, double vel_y, double yaw_rate)
{
    state curent;
    curent.x_pos = temp.x_pos + (vel_x*cos(temp.yaw) - vel_y*sin(temp.yaw))*dt;
    curent.y_pos = temp.y_pos + (vel_x*sin(temp.yaw) + vel_y*cos(temp.yaw))*dt;
    curent.yaw = temp.yaw + yaw_rate*dt;
    curent.x_vel = vel_x;
    curent.y_vel = vel_y;
    curent.yaw_rate = yaw_rate;

    if(curent.yaw < -1*M_PI)curent.yaw+=2*M_PI;

    if(curent.yaw > M_PI)curent.yaw-=2*M_PI;

    return curent;
}

bool dwa::is_valid(int x,int y)
{
    if(x<0 || y<0)return true;
    if(x>cur_map_f.info.width || y>cur_map_f.info.height)return true;
    if(cur_map_f.data[x*cur_map_f.info.width+y]==-1 || cur_map_f.data[x*cur_map_f.info.width+y]>=free_thresh)
        return true;
}

double dwa::traj_dist(state curent)
{
    double temp = curent.x_pos*cos(map_yaw) + curent.y_pos*sin(map_yaw);
    temp -= map_ori[0];
    int x_pos = temp/cur_map_f.info.resolution;

    temp = -curent.x_pos*sin(map_yaw) + curent.y_pos*cos(map_yaw); 
    temp -= map_ori[1];
    int y_pos = temp/cur_map_f.info.resolution;

    vector<point> visited;
    queue<point> unvisited;

    unvisited.push(point(x_pos,y_pos));
    int dist=0;
    while(!unvisited.empty())
    {
        point curr = unvisited.front();
        unvisited.pop();
        if(find(visited.begin(),visited.end(),curr) == visited.end())visited.push_back(curr);
        visited.push_back(curr);
        if(is_valid(curr.x,curr.y))return dist;
        for(int i=curr.x-1;i<=curr.x+1;i++)
        {
            for(int j=curr.y-1;j<=curr.y+1;j++)
            {
                if(i==curr.x && j==curr.y)continue;
                if(find(visited.begin(),visited.end(),point(i,j)) == visited.end())continue;
                if(is_valid(i,j))return dist+1;
                unvisited.push(point(i,j));
            }
        }
        dist++;
        if(dist>cons_rad)return cons_rad+1;
    }
    return dist;
}

double dwa::mini_dist(vector<state> trajectory)
{
    double min_dist=1e9;

    cur_map_f = cur_map;

    tf::Quaternion q;
    tf::quaternionMsgToTF(cur_map_f.info.origin.orientation,q);
    map_yaw = getYaw(q);

    map_ori[0]=cur_map_f.info.origin.position.x;
    map_ori[1]=cur_map_f.info.origin.position.y;

    for(vector<state>::iterator it=trajectory.begin();it!=trajectory.end();it++)
    {
        double temp = traj_dist(*it);
        if(temp < min_dist)min_dist = temp; 
    }

    return min_dist;
}

double dwa::heading_cost(state current)
{
    double theta = atan2(y_goal - current.y_pos,x_goal - current.x_pos);
    return abs(current.yaw - theta);
}


double dwa::distance_cost(state current, state previous)
{
    double now = distance(x_goal,y_goal,current.x_pos, current.y_pos);
    double prev = distance(x_goal,y_goal,previous.x_pos, previous.y_pos);
    if((prev - now)<0.01)return 100;
    return 1/(prev-now);
}

double dwa::velocity_cost(double vel)
{
    return (abs(max_speed_f) - abs(vel))/abs(max_speed_f);
}

vector< state > dwa::calc_trajectory (state init, double velocity, double omega)
{
    state temp=init;
    vector< state > trajectory;
    double time = 0;
    trajectory.push_back(init);
    while(time < cycle_time)
    {
        temp = motion(temp, velocity, 0, omega);
        trajectory.push_back(temp);
        time += dt;      
    }
    return trajectory;
}

vector<double> dwa::createDynamicWindow(state currentState)
{
    double tempMinspeed = currentState.x_vel - max_accel*dt;
    double tempMaxspeed = currentState.x_vel + max_accel*dt;
    double tempMinomega = currentState.yaw - max_yaw_acc*dt;
    double tempMaxomega = currentState.yaw + max_yaw_acc*dt;

    vector <double> DynamicWindow(4);

    DynamicWindow[0] = max(tempMinspeed,max_speed_b);
    DynamicWindow[1] = min(tempMaxspeed,max_speed_f);
    DynamicWindow[2] = max(tempMinomega,-1*max_yaw_rate);
    DynamicWindow[3] = min(tempMaxomega,max_yaw_rate);

    return DynamicWindow;
}

vector<double> dwa::path_find(state current,ros::NodeHandle n)
{
    vector<double> Dw = createDynamicWindow(current);
    vector<double> vel_ome(2);
    vel_ome[0] = -current.x_vel;

    if (!(n.getParam("cons_rad", cons_rad)))
        cons_rad=2.5/cur_map_f.info.resolution;
    else
        cons_rad/=cur_map_f.info.resolution;
    
    if (!(n.getParam("min_obj_dist", min_obj_dist)))
        min_obj_dist=0.5/cur_map_f.info.resolution;
    else
        min_obj_dist/=cur_map_f.info.resolution;

    double min_cost = 1e9;

    for(double vel=Dw[0];vel <= Dw[1];vel+=vel_reso)
    {
        for(double ome=Dw[2];ome <= Dw[3];ome+= yaw_reso)
        {
            vector<state> trajectory = calc_trajectory(current, vel, ome);

            double head_cost = heading_cost(trajectory.back());
            double vel_cost = velocity_cost(trajectory.back().x_vel);
            double min_dist = mini_dist(trajectory);
            double dist_cost = distance_cost(trajectory.back(),current);

            if(vel*vel >= 2*max_accel*(min_dist+min_obj_dist))
                continue;

            if(min_dist - min_obj_dist < 0)
                continue;

            double obj_cost;
            if(min_dist < cons_rad)
                obj_cost = 1/min_dist;
            else
                obj_cost = 0;

            double temp_cost = head_cost + dist_cost + vel_cost + obj_cost;
            
            if(temp_cost<min_cost && abs(vel)>vel_reso)
            {
                min_cost=temp_cost;
                vel_ome[0]=vel;
                vel_ome[1]=ome;
            } 
        }
    }

    return vel_ome;
}

