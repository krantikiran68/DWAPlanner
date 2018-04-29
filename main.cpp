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
        for(float ome = Dw[2];ome<=Dw[3];ome+=omega_reso)
        {
            vector<state> trajectory = calc_trajectory(x, vel, ome);
            
            float head_cost = (headingWeight) * heading_cost(trajectory.back());
            float vel_cost = (velocityWeight) * velocity_cost((trajectory.back()).vel);
            float obj_cost = (obstacleWeight) * obstacle_cost(trajectory, distance);

            float temp_cost = head_cost + vel_cost + obj_cost;
             
            if(temp_cost<min_cost)
            {
                min_cost = temp_cost;
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
    cvtColor(A,A,CV_BGR2GRAY);

    vector<vector<double> > distance = BFS(A);

    // Mat B(400,640,CV_8UC1,Scalar(0));
    // for(int i=0;i<A.rows;i++)
    // {
    //     for(int j=0;j<A.cols;j++)
    //     {
    //         if(distance[i][j]==0)
    //         {
    //             B.at<uchar>(i,j)=255;
    //         }
    //     }
    // }
    // imshow("win",B);
    // waitKey(0);

    current.x_pos = 19;
    current.y_pos = 381;
    current.omega = 0;
    current.theta = M_PI/4;
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
    for(vector<state>::iterator it=path.begin();it!=path.end();it++)
    {
        cout<<it->y_pos<<" "<<it->x_pos<<endl;
    }
    return 0;
}
