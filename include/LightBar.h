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


	//���أ�= ����
	//friend bool operator!=(const LightBar& t1, const LightBar& t2);

	//��������ĳ�����������Ƕȡ����ĵ�
	float height, width;
	float area;
	float angle;
	Point2f center;
};

#endif // 
