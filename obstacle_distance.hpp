#ifndef OBSDIS_H
#define OBSDIS_H

#include<bits/stdc++.h> 
#include<opencv2/opencv.hpp>
#include<windowparam.hpp>
using namespace std; 
using namespace cv;

int isValid(int i,int j,int r,int c);
vector<vector<vector<double> > > search(Mat img);
float BFS(Point bot,vector<vector<vector<double> > > distance);

#endif