#include <windowparam.hpp>


vector<float> createDynamicWindow(state currentState)
{
  /****************************************
  Location 0: Minimum speed
  Location 1: Maximum speed
  Location 2: Minimum omega
  Location 3: Maximum omega
  *****************************************/

  float tempMinspeed = currentState.vel - max_accel*dt;
  float tempMaxspeed = currentState.vel + max_accel*dt;
  float tempMinomega = currentState.omega - max_domega*dt;
  float tempMaxomega = currentState.omega + max_domega*dt;

  vector <float> DynamicWindow(4);

  DynamicWindow[0] = tempMinspeed>min_speed?tempMinspeed:min_speed;
  DynamicWindow[1] = tempMaxspeed<max_speed?tempMaxspeed:max_speed;
  DynamicWindow[2] = tempMinomega>-1*max_omega?tempMinomega:-1*(max_omega);
  DynamicWindow[3] = tempMaxomega<max_omega?tempMaxomega:max_omega;

  return DynamicWindow;
}

///////////////////HEADING COST///////////////////////

float heading_cost(state current)
{
  float cost;

  float dy= y_goal - current.y_pos;
  float dx= x_goal - current.x_pos;

  cost= sqrt(dy*dy + dx*dx);
  return cost;
}


//////////////////VELOCITY COST//////////////////////

float velocity_cost(float velocity)
{
  float cost;
  cost = max_speed - velocity;
  return cost;
}

//////////////////OBSTACLE COST//////////////////////

float obstacle_cost(vector<state> trajectory, vector<vector<double> > &distance)
{
  float min_dist=1e9;
  for(vector<state>::iterator it=trajectory.begin();it!=trajectory.end();it++)
  {
    if(distance[(int)(it->y_pos)][(int)(it->x_pos)]<min_dist)
    {
      min_dist = distance[(int)(it->y_pos)][(int)(it->x_pos)];
    }
  }
  float cost = min_dist==0?FLT_MAX:10/min_dist;
  return cost;
}

