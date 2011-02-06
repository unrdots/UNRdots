#include <ros/ros.h>
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
	ros::Time t1 = ros::Time::now();

	//convert the image to an openCV format
	frame = bridge.imgMsgToCv(image_message, "rgb8");
	IplImage *imgOut = cvCreateImage(cvGetSize(frame), frame->depth, frame->nChannels);
	cvCvtColor(frame,frame,CV_BGR2RGB);
	//segment here
	findColor(frame, imgOut, rCenter, cCenter);
	//draw a dot on the center of mass
	cvCircle(frame, cvPoint(cCenter,rCenter), 5, CV_RGB(0,150,200), -1);
	cvCircle(imgOut, cvPoint(cCenter,rCenter), 5, CV_RGB(255,150,200), -1);
	//display the image
	cvShowImage("Drone Camera",frame);
	cvShowImage("Segmented",imgOut);

	cvWaitKey(18);
	ROS_INFO("%f seconds",(ros::Time::now()-t1).toSec());
}


int main(int argc, char **argv){
	//start ros stuff
	ros::init(argc, argv, "ardrone_unr_team6");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/ardrone/image_raw",18, getFrame);

	ros::spin();

	return 0;
}
