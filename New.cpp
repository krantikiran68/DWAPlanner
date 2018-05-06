#include<bits/stdc++.h> 
#include<obstacle_distance.hpp>

float max_speed = 1.0  ;
float min_speed = -1.0  ;
float max_omega = 40.0 * M_PI / 180.0  ;
float max_accel = 0.3  ;
float max_domega = 40.0 * M_PI / 180.0  ;
float vel_reso = 0.05  ;
float omega_reso = 0.1 * M_PI / 180.0    ;
float predict_time = 12;
float x_goal = 361 ;
float y_goal = 19;
float dt = 0.6;
float goal_res = 1;
float inflt_radi = 3;
float cons_rad = 8;
float init_dist;

using namespace std;
using namespace cv;

struct state
{
    float x_pos, y_pos, theta, vel, omega;
};

float distance(float x1, float y1, float x2, float y2)
{
    return sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}

state motion(state temp, float velocity, float omega)
{
    state curent;
    curent.x_pos = temp.x_pos + velocity*cos(temp.theta)*dt;
    curent.y_pos = temp.y_pos + velocity*sin(temp.theta)*dt;
    curent.theta = temp.theta + omega*dt;
    curent.vel = velocity;
    curent.omega = omega;

    if(curent.theta < -1*M_PI)curent.theta+=2*M_PI;

    if(curent.theta > M_PI)curent.theta-=2*M_PI;

    return curent;
}

float obstacle_cost(vector<state> trajectory, vector< vector<double> > &distance)
{
    float min_dist=1e9;

    for(vector<state>::iterator it=trajectory.begin();it!=trajectory.end();it++)
    {
        if(distance[(int)(it->y_pos)][(int)(it->x_pos)]<min_dist)
            min_dist = distance[(int)(it->y_pos)][(int)(it->x_pos)];
    }

    return min_dist;
}

float heading_cost(state current)
{
    float theta = atan2(y_goal - current.y_pos,x_goal - current.x_pos);
    return abs(current.theta - theta);
}


float distance_cost(state current, state previous)
{
    float now = distance(x_goal,y_goal,current.x_pos, current.y_pos);
    float prev = distance(x_goal,y_goal,previous.x_pos, previous.y_pos);
    if((prev - now)<0.01)return 100;
    return 1/(prev-now);
}

float velocity_cost(float vel)
{
    return (abs(max_speed) - abs(vel))/abs(max_speed);
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

vector<float> createDynamicWindow(state currentState)
{
    float tempMinspeed = currentState.vel - max_accel*dt;
    float tempMaxspeed = currentState.vel + max_accel*dt;
    float tempMinomega = currentState.omega - max_domega*dt;
    float tempMaxomega = currentState.omega + max_domega*dt;

    vector <float> DynamicWindow(4);

    DynamicWindow[0] = max(tempMinspeed,min_speed);
    DynamicWindow[1] = min(tempMaxspeed,max_speed);
    DynamicWindow[2] = max(tempMinomega,-1*max_omega);
    DynamicWindow[3] = min(tempMaxomega,max_omega);

    return DynamicWindow;
}

vector<float> path_find(state current, vector< vector<double> > &distance)
{
    vector<float> Dw = createDynamicWindow(current);
    vector<float> vel_ome(2);
    vel_ome[0]=current.vel;


    float min_cost = 1e9;

    for(float vel=Dw[0];vel <= Dw[1];vel+=vel_reso)
    {
        for(float ome=Dw[2];ome <= Dw[3];ome+= omega_reso)
        {
            vector<state> trajectory = calc_trajectory(current, vel, ome);

            float head_cost = heading_cost(trajectory.back());
            float vel_cost = velocity_cost(trajectory.back().vel);
            float min_dist = obstacle_cost(trajectory, distance);
            float dist_cost = distance_cost(trajectory.back(),current);

            if(vel*vel >= 2*max_accel*(min_dist+inflt_radi))
                continue;

            if(min_dist < inflt_radi)
                continue;

            float obj_cost;
            if(min_dist < cons_rad)
                obj_cost = 1/min_dist;
            else
                obj_cost = 0;

            float temp_cost = head_cost + dist_cost + vel_cost + obj_cost;
            
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

int main()
{
    state current;

    current.x_pos = 19; 
    current.y_pos = 381;
    current.omega = 0;
    current.theta = 0;
    current.vel = 0;

    Mat A = imread("a.png",1);
    for(int i=0;i<A.rows;i++)
    {
        for(int j=0;j<A.cols;j++)
        {
            if(A.at<Vec3b>(i,j)[1] > A.at<Vec3b>(i,j)[0] && A.at<Vec3b>(i,j)[1] > A.at<Vec3b>(i,j)[2])
            {
                A.at<Vec3b>(i,j)[0]=0;
                A.at<Vec3b>(i,j)[1]=0;
                A.at<Vec3b>(i,j)[2]=0;
            }
            if(A.at<Vec3b>(i,j)[2] > A.at<Vec3b>(i,j)[0] && A.at<Vec3b>(i,j)[2] > A.at<Vec3b>(i,j)[1])
            {
                A.at<Vec3b>(i,j)[0]=0;
                A.at<Vec3b>(i,j)[1]=0;
                A.at<Vec3b>(i,j)[2]=0;
            }
        }
    }
    Mat C = A.clone();
    cvtColor(A,A,CV_BGR2GRAY);

    vector< vector<double> > distance = BFS(A);

    Mat B(400,640,CV_8UC1,Scalar(0));
    for(int i=0;i<A.rows;i++)
    {
        for(int j=0;j<A.cols;j++)
        {
            if(distance[i][j]==0)
            {
                B.at<uchar>(i,j)=255;
            }
            else if(distance[i][j] < 10)
            {
                B.at<uchar>(i,j)=175;
            }
            else if(distance[i][j] < 20)
            {
                B.at<uchar>(i,j)=125;
            }
        }
    }
    

    vector<state> path;
    path.push_back(current);

    float temp_x = current.x_pos - x_goal;
    float temp_y = current.y_pos - y_goal;
    init_dist = sqrt(temp_x*temp_x + temp_y*temp_y);

    while(1)
    {
        if(abs(current.x_pos - x_goal) < goal_res && abs(current.y_pos - y_goal) < goal_res)break;
        vector<float> vel_ome = path_find(current,distance);
        current = motion(current, vel_ome[0], vel_ome[1]);
        cout<<current.y_pos<<" "<<current.x_pos<<endl;  
        path.push_back(current);
        C.at<Vec3b>(current.y_pos,current.x_pos)[0]=0;
        C.at<Vec3b>(current.y_pos,current.x_pos)[1]=255;
        C.at<Vec3b>(current.y_pos,current.x_pos)[2]=0;
        imshow("win",C);
        // imshow("win1",B);
        waitKey(1);
    }

    // for(vector<state>::iterator it=path.begin();it!=path.end();it++)
    //     cout<<it->y_pos<<" "<<it->y_pos<<endl;

    return 0;
}