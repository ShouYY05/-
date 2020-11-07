#include "LightBar.h"

//�������캯��
LightBar::LightBar() {}

/**
* @brief �������캯������ʼ��������Ա����
*
* @param rect ������ת����
* @param area �������
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
