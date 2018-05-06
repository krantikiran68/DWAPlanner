#ifndef WINDOWPARAM_H
#define WINDOWPARAM_H
#include<bits/stdc++.h>
#include<trajectory.hpp>

using namespace std;

vector<float> createDynamicWindow(state currentState);
float heading_cost(state current);
float velocity_cost(float velocity);
float obstacle_cost(vector<state> trajectory, vector<vector<vector<double> > > distance);

#endif