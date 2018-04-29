#include<bits/stdc++.h> 
#include<obstacle_distance.hpp>

using namespace std;
using namespace cv;

vector<float> path_find(state x, vector<vector<double> > &distance)
{
    float min_cost = 1e9, best_vel, best_ome;

    vector<float> Dw = createDynamicWindow(x);
    for(float vel = Dw[0];vel<=Dw[1];vel+=vel_reso)
    {
        for(float ome = Dw[2];ome<=Dw[2];ome+=omega_reso)
        {
            vector<state> trajectory = calc_trajectory(x, vel, ome);
            
            float head_cost = (headingWeight) * heading_cost(trajectory.back());
            float vel_cost = (velocityWeight) * velocity_cost((trajectory.back()).vel);
            float obj_cost = (obstacleWeight) * obstacle_cost(trajectory, distance);

            float temp_cost = head_cost + vel_cost + obj_cost;

            if(temp_cost<min_cost)
            {
                min_cost=temp_cost;
                best_vel = vel;
                best_ome = ome;
            }
        }
    }

    vector<float> vel_ome(2);
    vel_ome[0] = best_vel;
    vel_ome[1] = best_ome;
    return vel_ome;
}

int main()
{
    state current;
    Mat A = imread("",CV_LOAD_IMAGE_GRAYSCALE);
    vector<vector<double> > distance = BFS(A);
    current.x_pos = 0;
    current.y_pos = 0;
    current.omega = M_PI/4;
    current.theta = 0;
    current.vel = 0;
    vector<state> path;
    path.push_back(current);
    while(1)
    {
        if(current.x_pos == x_goal && current.y_pos == y_goal)break;
        vector<float> vel_ome = path_find(current, distance);
        current = motion(current, vel_ome[0], vel_ome[1]);
        path.push_back(current);    
    }
    return 0;
}
