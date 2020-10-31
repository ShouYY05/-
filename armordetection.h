#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

#pragma once
#ifndef ARMORDETECTION_H
#define ARMORDETECTION_H

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class ArmorDetection {
private:
	Mat frame, hsv, mask;
	Mat kernel1 = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));
	Mat kernel2 = getStructuringElement(MORPH_RECT, Size(5, 5), Point(-1, -1));
	Point2f currentCenter;    //中心坐标
	Point2f lastCenter;
	float armor_area;         //面积
	float armor_angle;        //角度
	Point2f leftRect_up;
	Point2f rightRect_up;
	Point2f leftRect_down;
	Point2f rightRect_down;

	vector<RotatedRect> minRects;
	int lost = 0;

public:
	ArmorDetection();
	explicit ArmorDetection(Mat& input);
	void setInputImage(Mat input);
	void Pretreatment();
	Point2f GetArmorCenter();

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