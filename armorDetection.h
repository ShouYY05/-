#pragma once
#ifndef ARMOR_DETECTION_H
#define ARMOR_DETECTION_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "LightBar.h"
#include "armor.h"


using namespace std;
using namespace cv;

class ArmorDetection {
public:
	Mat frame, hsv, mask;
	Mat kernel1 = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));
	Mat kernel2 = getStructuringElement(MORPH_RECT, Size(5, 5), Point(-1, -1));
	Point2f currentCenter;    //ÖÐÐÄ×ø±ê
	Point2f lastCenter;
	
	int lost = 0;


	ArmorDetection();
	ArmorDetection(LightBar& left_light_bar, LightBar& right_light_bar);

	void setInputImage(Mat input);
	void Pretreatment(vector<LightBar> lights);
	void GetArmor(vector<LightBar> lights, vector<Armor> armor);

	int iLowH = 78;
	int iHighH = 255;

	int iLowS = 0;
	int iHighS = 255;

	int iLowV = 0;
	int iHighV = 255;

private:
	void LostTarget();
	double Distance(Point2f, Point2f);
	double max(double, double);
	double min(double, double);
};


#endif 
