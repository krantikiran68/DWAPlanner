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
  DynamicWindow[2] = tempMinomega>-1*max_omega?tempMinomega:max_omega;
  DynamicWindow[3] = tempMaxomega<max_omega?tempMaxomega:max_omega;

  return DynamicWindow;
}

///////////////////HEADING COST///////////////////////

float heading_cost(state current)
{
  float cost;

  float dy= y_goal - current.y_pos;
  float dx= x_goal - current.x_pos;

  cost= sqrt(dy*dy +dx*dx);
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

float obstacle_cost(state current)
{
  float cost;

  /*
  Method goes here
  */

  return cost;
}


//////////OBJECTIVE OPTIMIZATION///////////////////////

state CostOptimize(state current)
{

  vector <state> trajectory = calc_trajectory(current,current.vel,current.omega);

  float cost,mincost=FLT_MAX;

  state bestState;

  for(int i=0;i<trajectory.size();i++)
  {
     cost=0;

      vector <float> DynamicWindow = createDynamicWindow(trajectory[i]);

      for(float velocity=DynamicWindow[0];velocity<=DynamicWindow[1];velocity+=0.01)
      {
        for(float omega=DynamicWindow[2];omega<=DynamicWindow[3];omega+= 0.1*M_PI/180.0)
        {
          cost= (velocityWeight)*velocity_cost(velocity) + (headingWeight)*heading_cost(trajectory[i]) + (obstacleWeight)*obstacle_cost(trajectory[i]);
        }

      }

      if(mincost > cost)
      {
        mincost=cost;
        bestState=trajectory[i];
      }



  }

  return bestState;
}
