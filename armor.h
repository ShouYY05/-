#pragma once
#ifndef ARMOR_H
#define ARMOR_H

#include "LightBar.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "math.h"

class Armor
{
public:
	Armor(LightBar& left_light, LightBar& right_light);
	float armor_area;         //���
	float armor_angle;        //�Ƕ�
	float armor_height;        //�߶�
	float armor_width;        //���
	LightBar lights[2];             //�����ư����һ��װ�װ�

	//װ�װ���ĸ�����
	Point2f leftRect_up;
	Point2f rightRect_up;
	Point2f leftRect_down;
	Point2f rightRect_down;

	Point2f currentCenter;

};

RotatedRect tranform_rect(RotatedRect& rect);

double distance(Point2f p1, Point2f p2);

void getQuaternion(Mat R, double Q[]);

#endif // ! __ARMOR_


