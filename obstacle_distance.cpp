#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/video/tracking.hpp"
#include <iostream>
#include <queue>
#include <obstacle_distance.hpp>
using namespace std;
using namespace cv;

int isValid(int i,int j,int r,int c) {
	return (i >= 0 && j >= 0 && i < r && j <c);
}

vector<vector<vector<double> > > search(Mat img) {
	//preprocessing
	vector<vector<Point> > contours;
	vector<Vec4i> heirarchy;
	findContours(img, contours, heirarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

	vector<vector<vector<double> > > distance(contours.size(), vector<vector<double> >(img.rows, vector<double>(img.cols)));

	Mat visualise = img.clone();

	for(int i = 0;i < contours.size();i++) {

		vector<vector<int> > visited(img.rows,vector<int>(img.cols,0));
		queue<Point> q;

		for(int j = 0;j < contours[i].size();j++) {
			q.push(contours[i][j]);
			//cout << contours[i][j].y << " " << contours[i][j].x << endl;
			visited[contours[i][j].y][contours[i][j].x] = 1;
			distance[i][contours[i][j].y][contours[i][j].x] = 0.0;
		}	

		while(!q.empty()) {
		Point v = q.front();
		q.pop();

		for(int t = v.y-1;t<=v.y+1;t++) {
			for(int j = v.x-1;j <= v.x+1;j++) {
				if(isValid(t,j,img.rows,img.cols)) {
					if(visited[t][j] == 0 && visualise.at<uchar>(t,j) < 200) {
						//
						visualise.at<uchar>(t,j) = 150;
						//
						q.push(Point(j,t));
						visited[t][j] = 1;
						distance[i][t][j] = distance[i][v.y][v.x] + 1.0;
					}
				}
			}
		}
		}
	}

	return distance;

}

float BFS(Point bot,vector<vector<vector<double> > > distance) {

	int bot_x = (int)bot.x;
	int bot_y = (int)bot.y;

	float min_dist = FLT_MAX;
	//cout << distance.size() << endl;
	for(int i = 0;i < distance.size();i++) {
		if(distance[i][bot.y][bot.x] < min_dist) {
			min_dist = distance[i][bot.y][bot.x];
		}
	}

	return min_dist;

	// //push boundary
	// for(int i = 0;i<contours.size();i++) {
	// 	for(int j = 0;j<contours[i].size();j++) {
	// 		q.push(contours[i][j]);
	// 		visited[contours[i][j].y][contours[i][j].x] = 1;
	// 		distance[contours[i][j].y][contours[i][j].x] = 0.0;
	// 	}
	// }

	// while(!q.empty()) {
	// 	Point v = q.front();
	// 	q.pop();

	// 	for(int i = v.y-1;i<=v.y+1;i++) {
	// 		for(int j = v.x-1;j <= v.x+1;j++) {
	// 			if(isValid(i,j,img.rows,img.cols)) {
	// 				if(visited[i][j] == 0 && visualise.at<uchar>(i,j) < 200) {
	// 					//
	// 					visualise.at<uchar>(i,j) = 150;
	// 					//
	// 					q.push(Point(j,i));
	// 					visited[i][j] = 1;
	// 					distance[i][j] = distance[v.y][v.x] + 1.0;
	// 				}
	// 			}
	// 		}
	// 	}
			
	// }
}

// int main() {
// 	Mat A = imread("a.png",0);
// 	cout << BFS(Point(20,20),A) << endl;

// 	return 0;
// }

//driver function
// int main() {
// 	vector<vector<vector<double> > > distance = search(imread("a.png",0));
// 	cout << BFS(Point(0,0),distance) << endl;
// 	return 0;
// }