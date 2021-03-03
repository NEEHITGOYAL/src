/*
Copyright (c) 2011-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>
#include<iostream>
#include <find_object_2d/ObjectsStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <QTransform>
#include <QColor>
#include <sstream>
#include <find_object_2d/custom.h>
using namespace std;
image_transport::Publisher imagePub;

/**
 * IMPORTANT :
 *      Parameter General/MirrorView must be false
 *      Parameter Homography/homographyComputed must be true
 */
string name(int myid);

void objectsDetectedCallback(
		const std_msgs::Float32MultiArrayConstPtr & msg)
{
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<find_object_2d::custom>("box_coordinates", 1000);
	printf("---\n");
	const std::vector<float> & data = msg->data;
	if(data.size())
	{
		for(unsigned int i=0; i<data.size(); i+=12)
		{
			// get data
			int id = (int)data[i];
			float objectWidth = data[i+1];
			float objectHeight = data[i+2];

			// Find corners Qt
			QTransform qtHomography(data[i+3], data[i+4], data[i+5],
									data[i+6], data[i+7], data[i+8],
									data[i+9], data[i+10], data[i+11]);

			QPointF qtTopLeft = qtHomography.map(QPointF(0,0));
			QPointF qtTopRight = qtHomography.map(QPointF(objectWidth,0));
			QPointF qtBottomLeft = qtHomography.map(QPointF(0,objectHeight));
			QPointF qtBottomRight = qtHomography.map(QPointF(objectWidth,objectHeight));

			// printf("Object %d detected, Qt corners at (%f,%f) (%f,%f) (%f,%f) (%f,%f)\n",
			// 		id,
			// 		qtTopLeft.x(), qtTopLeft.y(),
			// 		qtTopRight.x(), qtTopRight.y(),
			// 		qtBottomLeft.x(), qtBottomLeft.y(),
			// 		qtBottomRight.x(), qtBottomRight.y());
			find_object_2d::custom msg2;
			msg2.id = id;
			msg2.top_left_x = qtTopLeft.x();
			msg2.top_left_y = qtTopLeft.y();	
			msg2.bottom_left_x = qtBottomLeft.x();
			msg2.bottom_left_y = qtBottomLeft.y();
			msg2.top_right_x = qtTopRight.x();
			msg2.top_right_y = qtTopRight.y();
			msg2.bottom_right_x = qtBottomRight.x();
			msg2.bottom_left_y = qtBottomRight.y();
			pub.publish(msg2);	
		}
	}
	// else
	// {
	// 	printf("No objects detected.\n");
	// }
}

void imageObjectsDetectedCallback(		const sensor_msgs::ImageConstPtr & imageMsg,		const find_object_2d::ObjectsStampedConstPtr & objectsMsg)
{
	// printf("Identification Started");
	if(imagePub.getNumSubscribers() >= 0)
	{
		const std::vector<float> & data = objectsMsg->objects.data;
		if(data.size())
		{
			for(unsigned int i=0; i<data.size(); i+=12)
			{
				// get data
				int id = (int)data[i];
				float objectWidth = data[i+1];
				float objectHeight = data[i+2];
                 
				// Find corners OpenCV
				cv::Mat cvHomography(3, 3, CV_32F);
				cvHomography.at<float>(0,0) = data[i+3];
				cvHomography.at<float>(1,0) = data[i+4];
				cvHomography.at<float>(2,0) = data[i+5];
				cvHomography.at<float>(0,1) = data[i+6];
				cvHomography.at<float>(1,1) = data[i+7];
				cvHomography.at<float>(2,1) = data[i+8];
				cvHomography.at<float>(0,2) = data[i+9];
				cvHomography.at<float>(1,2) = data[i+10];
				cvHomography.at<float>(2,2) = data[i+11];
				std::vector<cv::Point2f> inPts, outPts;
				inPts.push_back(cv::Point2f(0,0));
				inPts.push_back(cv::Point2f(objectWidth,0));
				inPts.push_back(cv::Point2f(objectWidth,objectHeight));
				inPts.push_back(cv::Point2f(0,objectHeight));
				inPts.push_back(cv::Point2f(objectWidth/2,objectHeight/2));
				cv::perspectiveTransform(inPts, outPts, cvHomography);

				cv_bridge::CvImageConstPtr imageDepthPtr = cv_bridge::toCvShare(imageMsg);

				cv_bridge::CvImage img;
				img = *imageDepthPtr;
				std::vector<cv::Point2i> outPtsInt;
				outPtsInt.push_back(outPts[0]);
				outPtsInt.push_back(outPts[1]);
				outPtsInt.push_back(outPts[2]);
				outPtsInt.push_back(outPts[3]);
				QColor color(QColor((Qt::GlobalColor)((id % 10 + 7)==Qt::yellow?Qt::darkYellow:(id % 10 + 7))));
				cv::Scalar cvColor(color.red(), color.green(), color.blue());
				cv::polylines(img.image, outPtsInt, true, (0,0,0), 2);
				cv::Point2i center = outPts[4];
				cv::putText(img.image,name(id) , outPts[3],  cv::FONT_HERSHEY_SIMPLEX, 0.8, (255,0,0), 2,cv::LINE_AA);
				cv::circle(img.image, center, 1, cvColor, 3);
				cv::Mat final =img.image;
				cv::cvtColor(img.image,final,CV_BGR2RGB);
				cv::imshow("Display",final);
                
				cv::waitKey(600);
				imagePub.publish(img.toImageMsg());
			}
		}
	}
}

typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, find_object_2d::ObjectsStamped> MyExactSyncPolicy;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "objects_detected");
    // printf("hi");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
	ros::Publisher pub = nh.advertise<find_object_2d::custom>("box_coordinates", 1000);
    imagePub = it.advertise("image_with_objects", 1000);
	// Simple subscriber
    ros::Subscriber sub;
    sub = nh.subscribe("objects", 1, objectsDetectedCallback);

    // Synchronized image + objects example
    image_transport::SubscriberFilter imageSub;
	imageSub.subscribe(it, nh.resolveName("/camera/color/image_raw2"), 1000);
	message_filters::Subscriber<find_object_2d::ObjectsStamped> objectsSub;
	objectsSub.subscribe(nh, "objectsStamped", 1);
    message_filters::Synchronizer<MyExactSyncPolicy> exactSync(MyExactSyncPolicy(1000), imageSub, objectsSub);
    exactSync.registerCallback(boost::bind(&imageObjectsDetectedCallback, _1, _2));

    

    ros::spin();

    return 0;
}
string name(int myid)
{
	int number;
	switch (myid)
	{
	case 143:
	//Coke
		number = 1;
		break;
	case 145:
	//Coke
		number = 1;
		break;	
	case 156:
	//Pair of Wheels Package
		number = 2;
		break;
	case 168:
	//Pair of Wheels Package
		number = 2;
		break;	
	case 158:
	//Pair of Wheels Package
		number = 2;
		break;	
	case 155:
	//FPGA Board
		number = 3;
		break;
	case 167:
	//FPGA Board
		number = 3;
		break;	
	case 149:
	//Glue
		number = 4;
		break;
	case 171:
	//Glue
		number = 4;
		break;	
	case 169:
	//Glue
		number = 4;
		break;	
	case 160:
	//Glue
		number = 4;
		break;	
	case 162:
	//Battery
		number = 5;
		break;
	case 157:
	//Battery
		number = 5;
		break;
	case 170:
	//Battery
		number = 5;
		break;		
	case 161:
	//eYFI Board
		number = 6;
		break;
	case 165:
	//eYFI Board
		number = 6;
		break;	
	case 164:
	//Glass
		number = 7;
		break;
	case 163:
	//Adhesive
		number = 8;
		break;
	case 159:
	//Adhesive
		number = 8;
		break;
	case 166:
	//Adhesive
		number = 8;
		break;	
	case 172:
	//Adhesive
		number = 8;
		break;											
	default:
		break;
	}
	string obj;
	switch (number)
	{
	case 1:
	    obj = "Coke";
		cout << "\033[37;42m"<< obj<<" identified " <<"\033[0m\n";
		break;
	case 2:
	    obj = "Pair of Wheels Package";
	    cout << "\033[37;42m"<< obj<<" identified " <<"\033[0m\n";
		break;
	case 3:
	    obj = "FPGA Board";
	    cout << "\033[37;42m"<< obj<<" identified " <<"\033[0m\n";
		break;
	case 4:
	    obj = "Glue";
	    cout << "\033[37;42m"<< obj<<" identified " <<"\033[0m\n";
		break;
	case 5:
	    obj = "Battery";
	    cout << "\033[37;42m"<< obj<<" identified " <<"\033[0m\n";
		break;
	case 6:
	    obj = "eYFI Board";
	    cout << "\033[37;42m"<< obj<<" identified " <<"\033[0m\n";
		break;
	case 7:
	    obj = "Glass";
	    cout << "\033[37;42m"<< obj<<" identified " <<"\033[0m\n";
		break;
	case 8:
	    obj = "Adhesive";
	    cout << "\033[37;42m"<< obj<<" identified " <<"\033[0m\n";
		break;							
	
	default:
		break;
	}
	return obj;
}
