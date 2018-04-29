#include <config_space.hpp>

using namespace std;
using namespace cv;

//Function to craete configuration space

//Mat src_img;
//int bot_length, bot_height;
void config_space(Mat src_img,int bot_height,int bot_length,Mat config[])
{
	
	Mat src_img_gry;
	
	cvtColor(src_img,src_img_gry,CV_BGR2GRAY);
	blur(src_img_gry,src_img_gry,Size(3,3));
	vector<vector<Point> > contours;
	findContours(src_img_gry,contours,CV_RETR_LIST,CV_CHAIN_APPROX_NONE,Point(0,0));
	for(int i=0;i<=360;i++)
	{
		config[i]=Mat::zeros(src_img.size(),CV_8U);
		
		for(size_t j=0;j<contours.size();j++)
		{
			drawContours(config[i],contours,j,255,CV_FILLED);
			for(size_t k=0;k<contours[j].size();k++)
				{
					RotatedRect R=RotatedRect(contours[j][k],Size2f(bot_length,bot_height),i);
					Point2f vertices2f[4];
					R.points(vertices2f);
					Point vertices[4];
					for(int i=0;i<4;i++)
						vertices[i]=vertices2f[i];
					fillConvexPoly(config[i],vertices,4,255);
				}
		}
		for (int j=0;j<src_img.rows;j++)
		{
			RotatedRect R=RotatedRect(Point(0,j),Size2f(bot_length,bot_height),i);
			Point2f vertices2f[4];
			R.points(vertices2f);
			Point vertices[4];
			for(int i=0;i<4;i++)
				vertices[i]=vertices2f[i];
			fillConvexPoly(config[i],vertices,4,255);

			 R=RotatedRect(Point(src_img.cols-1,j),Size2f(bot_length,bot_height),i);
			//Point2f vertices2f[4];
			R.points(vertices2f);
			//Point vertices[4];
			for(int i=0;i<4;i++)
				vertices[i]=vertices2f[i];
			fillConvexPoly(config[i],vertices,4,255);
		}

		for (int j=0;j<src_img.cols;j++)
		{
			RotatedRect R=RotatedRect(Point(j,0),Size2f(bot_length,bot_height),i);
			Point2f vertices2f[4];
			R.points(vertices2f);
			Point vertices[4];
			for(int i=0;i<4;i++)
				vertices[i]=vertices2f[i];
			fillConvexPoly(config[i],vertices,4,255);

			R=RotatedRect(Point(j,src_img.rows-1),Size2f(bot_length,bot_height),i);
			//Point2f vertices2f[4];
			R.points(vertices2f);
			//Point vertices[4];
			for(int i=0;i<4;i++)
				vertices[i]=vertices2f[i];
			fillConvexPoly(config[i],vertices,4,255);
		}

	}
	

}


/*int IsValid(Mat src_img,int bot_height,int bot_length,Point p)
{
	Mat config[361];
	config_space(src_img,bot_height,bot_length,config);
	int flag=1;
	if(p.x<0 || p.x>=src_img.cols)
	{
		//cout<<"Out of image" <<endl;
		return 0;
	}

	if(p.y<0 || p.y>=src_img.rows)
	{
		//cout<<"Out of image"<<endl;
		return 0;
	}

	for(int i=0;i<=360;i++)
	{
		if(config[i].at<uchar>(Point(p.x,p.y))==255)
		{
			//cout<< "Error at orientation "<<i<<endl;
			flag=0;
		}
		else
			cout<<"In position "<<i<<endl;
	}
	return flag;
}*/

int IsValid(Mat src_img,int bot_height,int bot_length,Point p,int theta)
{
	Mat config[361];
	config_space(src_img,bot_height,bot_length,config);
	//int flag=1;
	if(p.x<0 || p.x>=src_img.cols)
	{
		cout<<"Out of image" <<endl;
		return 0;
	}

	if(p.y<0 || p.y>=src_img.rows)
	{
		cout<<"Out of image"<<endl;
		return 0;
	}

	if(config[theta].at<uchar>(Point(p.x,p.y))==255)
	{
		return 0;
	}
	return 1;
}
