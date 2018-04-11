#include<obstacle_distance.hpp>

using namespace std;
using namespace cv;

int isValid(int i,int j,int r,int c) {
	return (i >= 0 && j >= 0 && i < r && j <c);
}

vector<vector<double> > BFS(Mat img) {
	vector<vector<double> > distance(img.rows,vector<double>(img.cols));
	vector<vector<int> > visited(img.rows,vector<int>(img.cols));

	for(int i = 0;i<img.rows;i++) {
		for(int j = 0;j<img.cols;j++) {
			visited[i][j] = 0;
		}
	}

	queue<Point> q;
	Mat visualise = img.clone();

	//preprocessing
	vector<vector<Point> > contours;
	vector<Vec4i> heirarchy;
	findContours(img, contours, heirarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

	//push boundary
	for(int i = 0;i<contours.size();i++) {
		for(int j = 0;j<contours[i].size();j++) {
			q.push(contours[i][j]);
			visited[contours[i][j].y][contours[i][j].x] = 1;
			distance[contours[i][j].y][contours[i][j].x] = 0.0;
		}
	}

	while(!q.empty()) {
		Point v = q.front();
		q.pop();

		for(int i = v.y-1;i<=v.y+1;i++) {
			for(int j = v.x-1;j <= v.x+1;j++) {
				if(isValid(i,j,img.rows,img.cols)) {
					if(visited[i][j] == 0 && visualise.at<uchar>(i,j) < 200) {
						//
						visualise.at<uchar>(i,j) = 150;
						//
						q.push(Point(j,i));
						visited[i][j] = 1;
						distance[i][j] = distance[v.y][v.x] + 1.0;
					}
				}
			}
		}
			
	}

	imshow("A",visualise);
	waitKey(0);

	return distance;
}
