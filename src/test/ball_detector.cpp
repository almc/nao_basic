/*
 * Copyright (c) 2012, 2013 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */
#include <iostream>
#include <unistd.h>
#include <math.h>

#include <alproxies/almotionproxy.h>
#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alimage.h>
#include <alvision/alvisiondefinitions.h>
#include <alerror/alerror.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/features2d/features2d.hpp>

const float img_HFOV = 47.64*M_PI/180.0;
const float img_WFOV = 60.97*M_PI/180.0;

int imageCount = 0;
float X_obj_old = 0; float Y_obj_old = 0;

int manual = 1;

int lowerH = 0;
int lowerS = 0;
int lowerV = 0;

int upperH = 180;
int upperS = 256;
int upperV = 256;

cv::SimpleBlobDetector* blobDetector;

void setupBlobDetector();
std::pair<int, int> GetThresholdedImage(cv::Mat& img_BGR, cv::Mat& img_THR,
                         int& HueLow, int& HueHigh, int& SatLow, int& SatHigh, int& ValueLow, int& ValueHigh,
                         cv::Scalar Color);



std::pair<float, float> worldBallPosFromImgCoords(AL::ALMotionProxy motionProxy, std::pair<int, int> ballPosCam, int imgWidth, int imgHeight, int camera) {
	std::string cameraName = "CameraTop";
	if (camera == 1) {
		cameraName = "CameraBottom";
	}

	int u = ballPosCam.first;
	int v = ballPosCam.second;

	float phi = ((float)u-((float)imgWidth)/2)/(float)imgWidth * img_WFOV;
	float theta = ((float)v-((float)imgHeight)/2)/(float)imgHeight * img_HFOV;

	//Select the right coordinates for the NAO system!
	//x outward from camera, y to the left and z vertically upwards

	//Coefficients for line-equation going from NAO camera to the ball
	float b_c = -sin(phi);
	float c_c = -sin(theta);
	float a_c = sqrt((cos(phi)*cos(phi)+cos(theta)*cos(theta))/2);


	int space = 2; //FRAME_ROBOT
  	bool useSensorValues = true;
  	std::vector<float> transVec = motionProxy.getTransform(cameraName, space, useSensorValues);
  	std::vector<float> cameraPos = motionProxy.getPosition(cameraName, space, useSensorValues);

/*
	std::cout << "Position of bottom camera: " << std::endl;
	std::cout << cameraPos.at(0) << " " << cameraPos.at(1) << " " << cameraPos.at(2) << std::endl;
	std::cout << cameraPos.at(3) << " " << cameraPos.at(4) << " " << cameraPos.at(5) << std::endl;
	*/

	//Put the camera transform into an Eigen matrix for easy multiplication
	Eigen::Matrix4f trans;
	trans << transVec[0], transVec[1], transVec[2], transVec[3],
		transVec[4], transVec[5], transVec[6], transVec[7],
		transVec[8], transVec[9], transVec[10], transVec[11],
		transVec[12], transVec[13], transVec[14], transVec[15];

	Eigen::Vector4f vec(a_c, b_c, c_c, 1);

	//Transform the line equation from NAO camera coordinate system into FRAME_ROBOT
	Eigen::Vector4f transformedLine = trans*vec;
	//std::cout << "trans*vec = " << transformedLine << std::endl;


	//Solve line-plane intersection with plane at z=0 (floor)
	Eigen::Matrix3f lineMat;
	lineMat << cameraPos.at(0)-transformedLine[0], 1-0, 0-0,
		cameraPos.at(1)-transformedLine[1], 0-0, 1-0,
		cameraPos.at(2)-transformedLine[2], 0-0, 0-0;

	Eigen::Vector3f lineVec;
	lineVec << cameraPos.at(0)-0.0, cameraPos.at(1)-0.0, cameraPos.at(2)-0.0;

	Eigen::Vector3f txy = lineMat.inverse()*lineVec;

	std::cout << "Intersection is at (x, y): (" << txy[1] << ", " << txy[2] << ")" << std::endl;

	std::pair<float, float> returnVal(txy[1], txy[2]);

	return returnVal;
}

void refreshImage(AL::ALVideoDeviceProxy &camProxy, std::string clientName, cv::Mat &img) {
	AL::ALValue alImg = camProxy.getImageRemote(clientName);

	/** Access the image buffer (6th field) and assign it to the opencv image
    * container. */
    img.data = (uchar*) alImg[6].GetBinary();

    /** Tells to ALVideoDevice that it can give back the image buffer to the
    * driver. Optional after a getImageRemote but MANDATORY after a getImageLocal.*/
    camProxy.releaseImage(clientName);
}

void setwindowSettings()
{
	cv::namedWindow("Display Window", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("Blob Window", CV_WINDOW_AUTOSIZE);

	cv::createTrackbar("Auto / Manual", "Blob Window", &manual, 1, NULL);

	cv::createTrackbar("LowerH", "Blob Window", &lowerH, 180, NULL);
	cv::createTrackbar("UpperH", "Blob Window", &upperH, 180, NULL);

	cv::createTrackbar("LowerS", "Blob Window", &lowerS, 256, NULL);
	cv::createTrackbar("UpperS", "Blob Window", &upperS, 256, NULL);

	cv::createTrackbar("LowerV", "Blob Window", &lowerV, 256, NULL);
	cv::createTrackbar("UpperV", "Blob Window", &upperV, 256, NULL);
}

int main() {
  std::cout << "Hello, world!" << std::endl;

  //std::string robotIP = "localhost";

  std::string robotIP = "192.168.0.195";

  AL::ALMotionProxy motionProxy(robotIP, 9559);
  AL::ALVideoDeviceProxy camProxy(robotIP, 9559);
  camProxy.setParam(AL::kCameraSelectID, 1);

  /** Subscribe a client image requiring 320*240 and BGR colorspace.*/
  const std::string clientName = camProxy.subscribe("test", AL::kQVGA, AL::kBGRColorSpace, 30);
  /** Create an cv::Mat header to wrap into an opencv image.*/
  cv::Mat image = cv::Mat(cv::Size(320, 240), CV_8UC3);

  //refreshImage(camProxy, clientName, imgHeader);

  setwindowSettings();

  setupBlobDetector();

  /** Main loop. Exit when pressing ESC.*/
  while ((char) cv::waitKey(30) != 27) {
    AL::ALValue img = camProxy.getImageRemote(clientName);
    image.data = (uchar*) img[6].GetBinary();
    camProxy.releaseImage(clientName);

	cv::Mat image_clone1 = image.clone();
	cv::Mat image_clone2 = image.clone();

	int red_hue_l = 0;
	int red_hue_h = 16;
	int red_sat_l = 121;
	int red_sat_h = 255;
	int red_val_l = 21;
	int red_val_h = 255;

	int green_hue_l = 40;
	int green_hue_h = 95;
	int green_sat_l = 33;
	int green_sat_h = 202;
	int green_val_l = 42;
	int green_val_h = 130;

	int blue_hue_l = 85;
	int blue_hue_h = 127;
	int blue_sat_l = 54;
	int blue_sat_h = 255;
	int blue_val_l = 65;
	int blue_val_h = 128;

	// std::cout << typeid(cv_ptr->image).name() << std::endl;


	std::pair<int, int> ballPosCam = GetThresholdedImage(image_clone1, image_clone2, red_hue_l, red_hue_h, red_sat_l, red_sat_h, red_val_l, red_val_h,
	                      CV_RGB(255,0,0));

	if (ballPosCam.first != -1)
	{
		std::pair<float, float> ballPosWorld = worldBallPosFromImgCoords(motionProxy, ballPosCam, 320, 240, 1);
		std::cout << "Ball pos in world: (" << ballPosWorld.first << ", " << ballPosWorld.second << ")" << std::endl;
	}

/*	GetThresholdedImage(image_clone1, image_clone2, green_hue_l, green_hue_h, green_sat_l, green_sat_h, green_val_l, green_val_h,
	                      CV_RGB(0,255,0));
	GetThresholdedImage(image_clone1, image_clone2, blue_hue_l, blue_hue_h, blue_sat_l, blue_sat_h, blue_val_l, blue_val_h,
	                      CV_RGB(0,0,255));
*/
	std::cout << "About to display images..." << std::endl;
	cv::imshow("Display Window"     , image);
	cv::imshow("Blob Window"        , image_clone1);
	std::cout << "...finished displaying" << std::endl;

  }


  delete blobDetector;

  camProxy.unsubscribe(clientName);
  return 0;
}

void setupBlobDetector() {
	// Set up the blob detection parameters
	cv::SimpleBlobDetector::Params params;
	params.minDistBetweenBlobs = 50.0f; // below this distance blobs will be merged
	params.filterByInertia = false;
	params.filterByConvexity = false;
	params.filterByColor = false;
	params.filterByCircularity = false;
	params.filterByArea = true;
	params.minArea = 100.0f;
	params.maxArea = 5000000.0f;
	// Set up and create the detector using the parameters
	// blobDetector(params);
	std::cout << "Before constructor call" << std::endl;
	blobDetector = new cv::SimpleBlobDetector(params); ///BAAAD GUY
	std::cout << "Between calls" << std::endl;
	blobDetector->create("SimpleBlob");

	std::cout << "Leaving method" << std::endl;
}


std::pair<int, int> GetThresholdedImage(cv::Mat& img_BGR, cv::Mat& img_THR,
                         int& HueLow, int& HueHigh, int& SatLow, int& SatHigh, int& ValueLow, int& ValueHigh,
                         cv::Scalar Color)
{
	std::cout << "GetThresholdedImage starting" << std::endl;
	cv::RNG rng(12345);
	// Convert the image into an HSV image
	cv::Mat img_HSV; cv::cvtColor(img_BGR, img_HSV, CV_BGR2HSV);

	if (manual)
		cv::inRange(img_HSV, cv::Scalar(lowerH, lowerS, lowerV), cv::Scalar(upperH, upperS, upperV), img_THR);
	else
		cv::inRange(img_HSV, cv::Scalar(HueLow, SatLow, ValueLow), cv::Scalar(HueHigh, SatHigh, ValueHigh), img_THR);

	int kernel_size = 5;
	cv::Mat kernel = cv::Mat::ones( kernel_size, kernel_size, CV_32F ) / (float)(kernel_size * kernel_size);
	cv::dilate(img_THR, img_THR, kernel, cv::Point(-1,-1), 3, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue());
	cv::erode (img_THR, img_THR, kernel, cv::Point(-1,-1), 4, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue());
	// cv::floodFill(img_THR, cv::Point(0, 0), cv::Scalar(0), NULL, cv::Scalar(20), cv::Scalar(20), 4);

	// Detect edges using canny
	cv::Mat canny_output; int thresh = 100;
	cv::Canny(img_THR, canny_output, thresh, thresh * 2, 3);
	// Find contours
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	// Aproximate contours
	std::vector<std::vector<cv::Point> > approxContours;
	approxContours.resize(contours.size());
	// Draw contours
	for (unsigned int i = 0; i < contours.size(); i++)
	{
		cv::Scalar color(rand()&255, rand()&255, rand()&255);
		// cv::drawContours(img_BGR, contours, i, color, CV_FILLED, 8, hierarchy );
		cv::drawContours(img_THR, contours, i,   255, CV_FILLED, 8, hierarchy );
	}
	cv::medianBlur(img_THR, img_THR, 5);

	// Blur image
	cv::GaussianBlur(img_THR, img_THR, cv::Size(7,7), 15000, 15000, cv::BORDER_DEFAULT);

	// Detect edges using Threshold
	cv::threshold(img_THR, img_THR, 100, 250, cv::THRESH_BINARY);

	// Find contours
	findContours(img_THR, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

	// Find the convex hull object for each contour
	std::vector<std::vector<cv::Point> > hull(contours.size());
	for(unsigned int i = 0; i < contours.size(); i++)
	{
		convexHull(cv::Mat(contours[i]), hull[i], false);
	}

	// Draw contours + hull results
	for(unsigned int i = 0; i< contours.size(); i++)
	{
		cv::Scalar color = cv::Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));
		drawContours(img_BGR, contours, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
		drawContours(img_THR,     hull, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
	}

	// for (unsigned int i = 0; i < contours.size(); i++)
	// {
	// 	approxPolyDP(cv::Mat(contours[i]), approxContours[i], 4, 1);
	// 	drawContours(img_BGR, contours      , i, CV_RGB(rand()&255, rand()&255, rand()&255) );
	// 	// drawContours(img_BGR, approxContours, i, CV_RGB(rand()&255, rand()&255, rand()&255) );
	// }

	// cv::Mat draw_contour = cv::Mat::zeros(canny_output.size(), CV_8UC3);
	// for (unsigned int i = 0; i < contours.size(); i++)
	// {
	// 	cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
	// 	cv::drawContours(img_BGR, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
	// }
/*
	// Set up the blob detection parameters
	cv::SimpleBlobDetector::Params params;
	params.minDistBetweenBlobs = 50.0f; // below this distance blobs will be merged
	params.filterByInertia = false;
	params.filterByConvexity = false;
	params.filterByColor = false;
	params.filterByCircularity = false;
	params.filterByArea = true;
	params.minArea = 100.0f;
	params.maxArea = 5000000.0f;
	// Set up and create the detector using the parameters
	cv::SimpleBlobDetector blobDetector(params); ///BAAAD GUY
	blobDetector.create("SimpleBlob");
*/	// Detect blobs
	std::vector<cv::KeyPoint> keyPoints;
	blobDetector->detect(img_THR, keyPoints);
	// Draw keypoints
	cv::drawKeypoints(img_BGR, keyPoints, img_BGR, CV_RGB(rand()&255, rand()&255, rand()&255), cv::DrawMatchesFlags::DEFAULT);

	float X_obj = 0; float Y_obj = 0;
	std::cout << "*Keypoints " << keyPoints.size() << "  *Contours  " << contours.size()  << std::endl;
	for (unsigned int i = 0; i < keyPoints.size(); i++)
	{
		float X = keyPoints[i].pt.x;
		float Y = keyPoints[i].pt.y;
		float R = keyPoints[i].size;
		circle(img_BGR, cv::Point(X, Y), R + 5, Color, 8, 0);
		X_obj += X; Y_obj += Y;
		std::cout << "  i: " << i << "  (X -|- Y) : (" << X << " -|- " << Y << ") Radius: " << R << std::endl;
	}
	X_obj /= keyPoints.size();
	Y_obj /= keyPoints.size();

	std::pair<int, int> returnVal(-1, -1);

	if (keyPoints.size() != 0)
	{
		X_obj_old = X_obj; Y_obj_old = Y_obj;
		returnVal.first = X_obj;
		returnVal.second = Y_obj;
		circle(img_BGR, cv::Point(X_obj, Y_obj), 5, CV_RGB(255,255,255), 4, 8, 0);
	}
	else
	{
		std::cout << "******************** NO BLOBS FOUND ********************" << std::endl;
		circle(img_BGR, cv::Point(X_obj_old, Y_obj_old), 5, CV_RGB(255,255,255), 4, 8, 0);
	}

	// std::cout << "Reached end of GetThresholdedImage" << std::endl;

//	sleep(5);

	return returnVal;
}
