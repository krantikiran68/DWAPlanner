#include<bits/stdc++.h> 
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

void config_space(Mat src_img,int bot_height,int bot_length,Mat config[]);
int IsValid(Mat src_img,int bot_height,int bot_length,Point p,int theta);