#include<obstacle_distance.hpp>

using namespace std;
using namespace cv;

int isValid(int i,int j,int r,int c) {
	return (i >= 0 && j >= 0 && i < r && j <c);
}

vector< vector<double> > BFS(Mat img) 
{
	vector< vector<double> > distance(img.rows,vector<double>(img.cols,FLT_MAX));

	queue<Point> q;
	Mat visualise = img.clone();

	//preprocessing
	vector<vector<Point> > contours;
	vector<Vec4i> heirarchy;
	findContours(img, contours, heirarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

	//push boundary
	for(int i = 0;i<contours.size();i++)
	{
		vector< vector<bool> > visited(img.rows,vector<bool>(img.cols,false));
		for(int j = 0;j<contours[i].size();j++) 
		{
			q.push(contours[i][j]);
			visited[contours[i][j].y][contours[i][j].x] = true;
			distance[contours[i][j].y][contours[i][j].x] = 0.0;
		}


		while(!q.empty()) 
		{
			Point v = q.front();
			q.pop();

			for(int i = v.y-1;i<=v.y+1;i++) 
			{
				for(int j = v.x-1;j <= v.x+1;j++) 
				{
					if(isValid(i,j,img.rows,img.cols)) 
					{
						if(!visited[i][j]) 
						{
							if(visualise.at<uchar>(i,j) > 100)
								distance[i][j]=0;
							else if(distance[i][j] > distance[v.y][v.x] + 1.0)
								distance[i][j] = distance[v.y][v.x] + 1.0;
							q.push(Point(j,i));
							visited[i][j] = 1;
						}
					}
				}
			}
				
		}
	}
	return distance;
}
