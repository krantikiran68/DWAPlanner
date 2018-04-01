#include<bits/stdc++.h>
#include<trajectory.hpp>

using namespace std;

state motion(state temp, float velocity, float omega)
{
    state curent;
    curent.x_pos = temp.x_pos + velocity*cos(temp.theta)*dt;
    curent.y_pos = temp.y_pos + velocity*sin(temp.theta)*dt;
    curent.theta = omega*dt;
    curent.vel = velocity;
    curent.omega = omega;

    return temp;
}

vector< state > calc_trajectory (state init, float velocity, float omega)
{
    state temp=init;
    vector< state > trajectory;
    float time = 0;
    trajectory.push_back(init);
    while(time < predict_time)
    {
        temp = motion(temp, velocity, omega);
        trajectory.push_back(temp);
        time += dt;      
    }
    return trajectory;
}