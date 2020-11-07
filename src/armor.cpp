#include "LightBar.h"
#include "Armor.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "math.h"


Armor::Armor(LightBar& left_light_bar, LightBar& right_light_bar)
{


	//定义左、右灯条的4个顶点
	Point2f left_light_vertices[4];
	Point2f right_light_vertices[4];
	left_light_bar.rect.points(left_light_vertices);
	right_light_bar.rect.points(right_light_vertices);

	//初始化成员函数
	this->lights[0] = left_light_bar;
	this->lights[1] = right_light_bar;
	this->armor_height = (left_light_bar.height + right_light_bar.height) / 2;
	this->armor_width = abs(left_light_bar.center.x - right_light_bar.center.x) + left_light_bar.width;
	this->armor_area = this->armor_height * this->armor_width;

	//将光条中的边界点传给装甲板的四个顶点
	this->leftRect_down = left_light_vertices[0];     //注意点的顺序
	this->leftRect_up = left_light_vertices[1];       //*** 2 绿 **** 3 黄 ***
	this->rightRect_up = right_light_vertices[2];     //**********************
	this->rightRect_down = right_light_vertices[3];   //*** 1 青 **** 4 蓝 ***
	this->currentCenter.x = (left_light_vertices[0].x + left_light_vertices[1].x + left_light_vertices[2].x + left_light_vertices[3].x) / 4;
	this->currentCenter.y = (left_light_vertices[0].y + left_light_vertices[1].y + left_light_vertices[2].y + left_light_vertices[3].y) / 4;
}
