#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <queue>
#include <math.h>

using namespace std;

IplImage *frame;
IplImage *imgOut;
sensor_msgs::CvBridge bridge;

ros::Subscriber sub;
ros::Publisher cmdPublisher;
geometry_msgs::Twist cmd_vel;

float xAngle;
float yAngle;
float heightDifference;
float heightThresh = .1;
float xAngleThresh = 1;
float yAngleThresh = 1;


//x and y are percent of fov
void calcAngles( int r, int c, float &xAngle, float &yAngle )
{
	if (r != -1)
	{
		//Rotate about the y-axis by roll
		float roll = 0;
		float r_temp = 320 / 2.0 + (c - 320 / 2.0) * sin(roll) + (r - 240 / 2.0) * cos(roll);
		float c_temp = 320 / 2.0 + (c - 320 / 2.0) * cos(roll) + (240 / 2.0 - r) * sin(roll);

		//Shift y angle by pitch
		float pitch = 0;
		r_temp += tan(pitch) * 240 / (2 * tan(90 / 2));

		//Convert pixel coordinates to percentages
		xAngle = (2 * c_temp / 320 - 1);
		yAngle = (1 - 2 * r_temp / 240);
	} else
	{
		xAngle = 0;
		yAngle = 0;
	}
}

void adjustDrone(int rCenter, int cCenter)
{
	///Computer Vision//////////////////////////////////////////////////////////
	//Calculate the angle from the center of the frame
	calcAngles(rCenter, cCenter, xAngle, yAngle);

	///Drone Communication///////////////////////////////////////////////////////////////////////// 
	//All x & y values should be zero at all times for the demo of the prototype
	//These values should change during the final project
	cmd_vel.linear.x  = 0;//max(min(-msg->linear.x, maxHorizontalSpeed), -maxHorizontalSpeed);
	cmd_vel.linear.y  = 0;//max(min(-msg->linear.y, maxHorizontalSpeed), -maxHorizontalSpeed);
	cmd_vel.angular.x = 0;
	cmd_vel.angular.y = 0;
	
	//turn counterclockwise by xAngle (z rotation)
	xAngle *= -1;
	cmd_vel.angular.z = xAngle;//max(min(-msg->angular.z, 1), -1);

	cmdPublisher.publish ( cmd_vel );//publish commands to drone
}

void findColor(IplImage* frameIn, IplImage* segFrame, int &r, int &c)
{
	int i,j;

	int threshHi = 50;
	int threshLo = 25;

	queue<int> rQueue;
	queue<int> cQueue;

	int rTmp, cTmp;

	int rSumCurrent = 0;
	int cSumCurrent = 0;

	int rSumMax = 0;
	int cSumMax = 0;

	int maxClusterSize = 0;
	int maxClusterNumber = 2;

	int w,h,ch,step;
	uchar *dataIn;
	uchar *dataOut;

	w = frameIn->width;
	h = frameIn->height;
	ch = frameIn->nChannels;
	step = frameIn->widthStep;

	dataIn = (uchar*) frameIn->imageData;
	dataOut = (uchar*) segFrame->imageData;

	//Initial thresholding of all pixels
	for(i=0; i<h; i++)
		for(j=0; j<w; j++)
		{
			dataOut[i*step + j*ch + 0] = 0;

			if(dataIn[i*step + j*ch + 2] -
					dataIn[i*step + j*ch + 1] -
					dataIn[i*step + j*ch + 0] > threshHi)
			{
				rQueue.push(i);
				cQueue.push(j);
				dataOut[i*step + j*ch + 0] = 1;
			}
		}

	//Apply Histeria thresholding (a lower threshold to points near points that passed the previous high threshold)
	while(!rQueue.empty())
	{
		rTmp = rQueue.front();
		rQueue.pop();
		cTmp = cQueue.front();
		cQueue.pop();

		for (i = rTmp - 1; i <= rTmp + 1; i++)
			for (j = cTmp - 1; j <= cTmp + 1; j++)
			{
				if (!(i < 0 || i >= h || j < 0 || j >= w))
					if (dataOut[i*step + j*ch + 0] != 1)
					{
						if (dataIn[i*step + j*ch + 2] -
								dataIn[i*step + j*ch + 1] -
								dataIn[i*step + j*ch + 0] > threshLo)
						{
							rQueue.push(i);
							cQueue.push(j);
							dataOut[i*step + j*ch + 0] = 1;
						}

					}
			}

	}

	//Find the largest cluster
	int cluster = 2;
	int currentClusterSize;
	for (i=0; i<h; i++)
		for (j=0; j<w; j++)
		{
			if (dataOut[i*step + j*ch + 0] == 1)
			{
				rQueue.push(i);
				cQueue.push(j);
				dataOut[i*step + j*ch + 0] = cluster;
				currentClusterSize = 0;
				rSumCurrent = 0;
				cSumCurrent = 0;

				while(!rQueue.empty())
				{
					rTmp = rQueue.front();
					rQueue.pop();
					cTmp = cQueue.front();
					cQueue.pop();

					rSumCurrent += rTmp;
					cSumCurrent += cTmp;

					currentClusterSize++;

					for (int y=rTmp-1; y<=rTmp+1; y++)
						for (int x=cTmp-1; x<=cTmp+1; x++)
						{
							if (!(y < 0 || y >= h || x < 0 || x >= w))
								if (dataOut[y*step + x*ch + 0] == 1)
								{
									rQueue.push(y);
									cQueue.push(x);
									dataOut[y*step + x*ch + 0] = cluster;
								}
						}
				}

				if (maxClusterSize < currentClusterSize)
				{
					maxClusterSize = currentClusterSize;
					maxClusterNumber = cluster;
					rSumMax = rSumCurrent;
					cSumMax = cSumCurrent;
				}

				cluster++;
			}
		}

	//Set values of cluster image
	for(i=0; i<h; i++)
		for(j=0; j<w; j++)
		{
			if(dataOut[i*step + j*ch + 0] == maxClusterNumber)
			{
				dataOut[i*step + j*ch + 0] = dataIn[i*step + j*ch + 0];
				dataOut[i*step + j*ch + 1] = dataIn[i*step + j*ch + 1];
				dataOut[i*step + j*ch + 2] = dataIn[i*step + j*ch + 2];
			}
			else
			{
				dataOut[i*step + j*ch + 0] = 0;
				dataOut[i*step + j*ch + 1] = 0;
				dataOut[i*step + j*ch + 2] = 0;

			}
		}

	//Set cluster center values
	if (maxClusterSize != 0)
	{
		r = rSumMax / maxClusterSize;
		c = cSumMax / maxClusterSize;
	}else
	{
		r = -1;
		c = -1;
	}

}


void getFrame(sensor_msgs::Image::ConstPtr image_message){
	int rCenter, cCenter;
	//time the execution of this call back
	//ros::Time t1 = ros::Time::now();

	//convert the image to an openCV format
	frame = bridge.imgMsgToCv(image_message, "rgb8");
	IplImage *imgOut = cvCreateImage(cvGetSize(frame), frame->depth, frame->nChannels);
	cvCvtColor(frame,frame,CV_BGR2RGB);
	//segment here
	findColor(frame, imgOut, rCenter, cCenter);
	adjustDrone(rCenter, cCenter);
	//draw a dot on the center of mass
	cvCircle(frame, cvPoint(cCenter,rCenter), 5, CV_RGB(0,150,200), -1);
	cvCircle(imgOut, cvPoint(cCenter,rCenter), 5, CV_RGB(255,150,200), -1);
	//display the image
	cvShowImage("Drone Camera",frame);
	cvShowImage("Segmented",imgOut);

	cvReleaseImage(&imgOut);

	cvWaitKey(18);

	//ROS_INFO("%f seconds",(ros::Time::now()-t1).toSec());
}


int main(int argc, char **argv){
	//start ros stuff
	ros::init(argc, argv, "ardrone_unr_team6");
	ros::NodeHandle n;
	ros::Rate loop_rate(30);
	sub = n.subscribe("/ardrone/image_raw",18, getFrame);
	cmdPublisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);

	ros::spin();

	return 0;
}
