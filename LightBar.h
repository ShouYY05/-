#pragma once
#ifndef LIGHT_BAR_H
#define LIGHT_BAR_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;

class LightBar
{
public:
	LightBar();
	LightBar(RotatedRect rect, float area);

	RotatedRect rect;


	//重载！= 符号
	//friend bool operator!=(const LightBar& t1, const LightBar& t2);

	//定义灯条的长、宽、面积、角度、中心点
	float height, width;
	float area;
	float angle;
	Point2f center;
};

#endif // 
