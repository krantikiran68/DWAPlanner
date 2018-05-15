#include<bits/stdc++.h> 

double max_speed = 1.0  ;
double min_speed = -1.0  ;
double max_omega = 40.0 * M_PI / 180.0  ;
double max_accel = 0.3  ;
double max_domega = 40.0 * M_PI / 180.0  ;
double vel_reso = 0.05  ;
double omega_reso = 0.1 * M_PI / 180.0    ;
double predict_time = 5;
double x_goal = 361 ;
double y_goal = 19;
double dt = 0.1;
double goal_res = 1;
double cons_rad = 8;

using namespace std;

double distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}

state motion(state temp, double vel_x, double vel_y, double yaw_rate)
{
    state curent;
    curent.x_pos = temp.x_pos + (vel_x*cos(temp.yaw) - vel_y*sin(temp.yaw))*dt;
    curent.y_pos = temp.y_pos + (vel_x*sin(temp.yaw) + vel_y*cos(temp.yaw))*dt;
    curent.yaw = temp.yaw + yaw_rate*dt;
    curent.vel_x = vel_x;
    curent.vel_y = vel_y;
    curent.yaw_rate = yaw_rate;

    if(curent.yaw < -1*M_PI)curent.yaw+=2*M_PI;

    if(curent.yaw > M_PI)curent.yaw-=2*M_PI;

    return curent;
}

double min_dist(vector<state> trajectory, vector< vector<double> > &distance)
{
    double min_dist=1e9;

    for(vector<state>::iterator it=trajectory.begin();it!=trajectory.end();it++)
    {
        if(distance[(int)(it->y_pos)][(int)(it->x_pos)]<min_dist)
            min_dist = distance[(int)(it->y_pos)][(int)(it->x_pos)];
    }

    return min_dist;
}

double heading_cost(state current)
{
    double theta = atan2(y_goal - current.y_pos,x_goal - current.x_pos);
    return abs(current.yaw - theta);
}


double distance_cost(state current, state previous)
{
    double now = distance(x_goal,y_goal,current.x_pos, current.y_pos);
    double prev = distance(x_goal,y_goal,previous.x_pos, previous.y_pos);
    if((prev - now)<0.01)return 100;
    return 1/(prev-now);
}

double velocity_cost(double vel)
{
    return (abs(max_speed) - abs(vel))/abs(max_speed);
}

vector< state > calc_trajectory (state init, double velocity, double omega)
{
    state temp=init;
    vector< state > trajectory;
    double time = 0;
    trajectory.push_back(init);
    while(time < predict_time)
    {
        temp = motion(temp, velocity, 0, omega);
        trajectory.push_back(temp);
        time += dt;      
    }
    return trajectory;
}

vector<double> createDynamicWindow(state currentState)
{
    double tempMinspeed = currentState.vel - max_accel*dt;
    double tempMaxspeed = currentState.vel + max_accel*dt;
    double tempMinomega = currentState.omega - max_domega*dt;
    double tempMaxomega = currentState.omega + max_domega*dt;

    vector <double> DynamicWindow(4);

    DynamicWindow[0] = max(tempMinspeed,min_speed);
    DynamicWindow[1] = min(tempMaxspeed,max_speed);
    DynamicWindow[2] = max(tempMinomega,-1*max_omega);
    DynamicWindow[3] = min(tempMaxomega,max_omega);

    return DynamicWindow;
}

vector<double> path_find(state current, vector< vector<double> > &distance)
{
    vector<double> Dw = createDynamicWindow(current);
    vector<double> vel_ome(2);
    vel_ome[0] = -current.vel_x;


    double min_cost = 1e9;

    for(double vel=Dw[0];vel <= Dw[1];vel+=vel_reso)
    {
        for(double ome=Dw[2];ome <= Dw[3];ome+= omega_reso)
        {
            vector<state> trajectory = calc_trajectory(current, vel, ome);

            double head_cost = heading_cost(trajectory.back());
            double vel_cost = velocity_cost(trajectory.back().vel_x);
            double min_dist = min_dist(trajectory, distance);
            double dist_cost = distance_cost(trajectory.back(),current);

            if(vel*vel >= 2*max_accel*min_dist)
                continue;

            if(!min_dist)
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

