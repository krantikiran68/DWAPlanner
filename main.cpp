#include<bits/stdc++.h> 
#include<obstacle_distance.hpp>

using namespace std;
using namespace cv;

vector<float> path_find(state x, vector<vector<double> > distance)
{
    float min_cost = 1e9, best_vel = 0.1, best_ome = 0.1;

    vector<float> Dw = createDynamicWindow(x);
    for(float vel = Dw[0];vel<=Dw[1];vel+=vel_reso)
    {
        for(float ome = Dw[2];ome<=Dw[3];ome+=omega_reso)
        { 
            vector<state> trajectory = calc_trajectory(x, vel, ome);
            
            float head_cost = (headingWeight) * heading_cost(trajectory.back());
            float vel_cost = (velocityWeight) * velocity_cost((trajectory.back()).vel);
            //float obj_cost = (obstacleWeight) * obstacle_cost(trajectory, distance);

            //cout << head_cost << " " << vel_cost << endl;
            float temp_cost = head_cost + vel_cost;
             
            if(temp_cost<min_cost)
            {
                min_cost = temp_cost;
                best_vel = vel;
                best_ome = ome;
            }
        }
    }
    //cout << "T" << endl;
    vector<float> vel_ome(2);
    vel_ome[0] = best_vel;
    vel_ome[1] = best_ome;
    return vel_ome;
}

int main()
{
    state current;
    Mat A = imread("a.png");
    cout << A.rows << endl << A.cols << endl;
    // for(int i=0;i<A.rows;i++)
    // {
    //     for(int j=0;j<A.cols;j++)
    //     {
    //         if(A.at<Vec3b>(i,j)[1] > A.at<Vec3b>(i,j)[0] && A.at<Vec3b>(i,j)[1] > A.at<Vec3b>(i,j)[2])
    //         {
    //             A.at<Vec3b>(i,j)[0]=0;
    //             A.at<Vec3b>(i,j)[1]=0;
    //             A.at<Vec3b>(i,j)[2]=0;
    //         }
    //         if(A.at<Vec3b>(i,j)[2] > A.at<Vec3b>(i,j)[0] && A.at<Vec3b>(i,j)[2] > A.at<Vec3b>(i,j)[1])
    //         {
    //             A.at<Vec3b>(i,j)[0]=0;
    //             A.at<Vec3b>(i,j)[1]=0;
    //             A.at<Vec3b>(i,j)[2]=0;
    //         }
    //     }
    // }
    cvtColor(A,A,CV_BGR2GRAY);

    vector<vector<double> > distance = BFS(A);
    cout << distance[A.rows-1][A.cols-1] << endl;
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

    current.x_pos = 1;
    current.y_pos = 1;
    current.omega = 0;
    current.theta = M_PI/4;
    current.vel = 0;
    vector<state> path;
    path.push_back(current);
    int i = 0;
    while(1)
    {
        float dis = sqrt((pow(current.x_pos-x_goal,2) + pow(current.y_pos-y_goal,2)));
        if(dis <= 1.0)break;
        vector<float> vel_ome = path_find(current, distance);
        current = motion(current, vel_ome[0], vel_ome[1]);
        path.push_back(current);    
        //cout << current.x_pos << " " << current.y_pos << " " << current.vel << " " << current.omega << endl;
    }
    for(vector<state>::iterator it=path.begin();it!=path.end();it++)
    {
        cout<<it->x_pos<<" "<<it->y_pos<<endl;
    }
    imshow("A",A);
    waitKey(0);
    return 0;
}
