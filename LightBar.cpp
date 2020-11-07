#include "LightBar.h"

//灯条构造函数
LightBar::LightBar() {}

/**
* @brief 灯条构造函数，初始化灯条成员变量
*
* @param rect 灯条旋转矩形
* @param area 灯条面积
* @param perimeter
*/
LightBar::LightBar(RotatedRect rect, float area)
{
	this->width = rect.size.width;
	this->height = rect.size.height;
	this->angle = rect.angle;
	this->center = rect.center;
	this->area = area;
}
