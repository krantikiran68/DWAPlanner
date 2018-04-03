#include<bits/stdc++.h>
#include<trajectory.hpp>

using namespace std;

vector<float> createDynamicWindow(state currentState);
float heading_cost(state current);
float velocity_cost(float velocity);
float obstacle_cost(state current);
state CostOptimize(state current);
