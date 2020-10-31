#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/imgproc/types_c.h>   //ɫ�ʿռ�ת��
#include <vector>                      //����
#include <stdlib.h>
#include "armordetection.h"


ArmorDetection::ArmorDetection() = default;

ArmorDetection::ArmorDetection(Mat & input) {
	frame = input;
}

void ArmorDetection::setInputImage(Mat input)                  //���뵱ǰͼ��
{
	frame = input;
	currentCenter.x = 0;    //��������������
	currentCenter.y = 0;
	leftRect_up.x = 0;      //�������Ϸ��е�  
	leftRect_up.y = 0;
	leftRect_down.x = 0;    //�������·��е�  
	leftRect_down.y = 0;
	rightRect_up.x = 0;     //�Ҳ�����Ϸ��е�  
	rightRect_up.y = 0;
	rightRect_down.x = 0;   //�Ҳ�����·��е�  
	rightRect_down.y = 0;

}

//ͼ��Ԥ������ͼ��ת��ΪHSV���ͣ������ýǵ��ҳ���Ե���õ���С��Ӿ��Σ���������Ե�ߺ�ͼ������λ�á�
void ArmorDetection::Pretreatment() {
	Mat input;
	Point p, center;
	vector<vector<Point>> contours;
	vector<Vec4i> hireachy;
	vector<Rect> boundRect(contours.size());
	Point2f vertex[4];                                         //��Ӿ��ε��ĸ�����

	imshow("src", frame);
	/*
	//����������
	createTrackbar("LowH", "Control", &iLowH, 255);
	createTrackbar("HighH", "Control", &iHighH, 255);

	createTrackbar("LowS", "Control", &iLowS, 255);
	createTrackbar("HighS", "Control", &iHighS, 255);

	createTrackbar("LowV", "Control", &iLowV, 255);
	createTrackbar("HighV", "Control", &iHighV, 255);
	cvtColor(frame, hsv, CV_BGR2HSV);
	inRange(hsv,
		Scalar(iLowH, iLowS, iLowV),
		Scalar(iHighH, iHighS, iHighV),
		mask);
	// ��̬ѧ����
	morphologyEx(mask, mask, MORPH_OPEN, kernel1, Point(-1, -1));//������
	dilate(mask, mask, kernel2, Point(-1, -1), 1);//����
	//������ǿ
	Canny(mask, mask, 3, 9, 3);
	imshow("mask", mask);
	*/

	cvtColor(frame, hsv, CV_BGR2HSV);                         //�Ѷ�ȡ��srcͼ��תΪHSVͼ
	vector<Mat> channels;                                     //�ֳ�����Ƶ��
	split(frame, channels);
	Mat hue = channels.at(0);
	Mat value = channels.at(2);
	Mat mask(hue.rows, hue.cols, CV_8UC1, Scalar(0, 0, 0));   //��������ͼ�񻭰�

	for (int i = 0; i < hue.rows; i++)
	{
		for (int j = 0; j < hue.cols; j++)
		{
			int h = hue.at<uchar>(i, j);                      //�õ�H��V��ֵ
			int v = value.at<uchar>(i, j);
			if (h > 78 && v < 50)                             //ɸѡ��ɫ����Ϊ��ɫֵ
				mask.at<uchar>(i, j) = 255;
		}
	}

	imshow("��ֵ��Եͼ", mask);
	findContours(mask, contours, hireachy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	//ɸѡ��ȥ��һ���־���
	for (int i = 0; i < contours.size(); ++i) 
	{
		RotatedRect minRect = minAreaRect(Mat(contours[i]));  //��С��Ӿ���

		minRect.points(vertex);                               //��Ӿ��ε��ĸ�����
		if (minRect.size.width > minRect.size.height)         //�������εĿ�͸ߣ�ʹ�ÿ���
		{
			minRect.angle += 90;
			float t = minRect.size.width;
			minRect.size.width = minRect.size.height;
			minRect.size.height = t;
		}

        //����ɸѡ���������ľ���,�����浽minRects��
		if ((minRect.size.width * 9 > minRect.size.height)       //���ο�ȸ�
			&& (minRect.size.width * 1 < minRect.size.height)
			&& (minRect.size.width * minRect.size.height < 160) && (minRect.size.width* minRect.size.height > 20) //�������>20, <160
			&& (abs(minRect.angle) < 30) )                        //����ƫ���
		{
			minRects.push_back(minRect);
		}

        /*//����Ȧ��ʶ�����С���Σ���ɫ���� ����ͼ����㣬�յ㣬��ɫ���߿�
		for (int l = 0; l < 4; l++)
		{
			line(frame, vertex[l], vertex[(l + 1) % 4], Scalar(255, 255, 255), 2);   
		}
		//����ͼ������ʮ��
		line(frame, Point(frame.cols / 2 - 15, frame.rows / 2),
			Point(frame.cols / 2 + 15, frame.rows / 2), Scalar(0, 255, 255), 5);
		line(frame, Point(frame.cols / 2, frame.rows / 2 - 15),
			Point(frame.cols / 2, frame.rows / 2 + 15), Scalar(0, 255, 255), 5);
		circle(frame, Point(frame.cols / 2, frame.rows / 2), 4, Scalar(0, 0, 255), -1);
		*/
	}
}


Point2f ArmorDetection::GetArmorCenter() {
	//�������о��Σ��������
	RotatedRect leftRect, rightRect;  //������������
	vector<int*> reliability;
	double area[2], distance, height;
	

	if (minRects.size() < 2)          //ֻ�ҵ�һ�����λ���û�о���
	{
		LostTarget();
		return currentCenter;
	}

	for (int i = 0; i < minRects.size(); ++i) 
	{
		for (int j = i + 1; j < minRects.size(); ++j) 
		{
			int num = 1;    //ͬһ��ͼ�ϵĲ�ͬ���ĵ�
			int level = 0;
			int temp[3];
			leftRect = minRects[i];        //�������о��Σ��������
			rightRect = minRects[j];
			//�ж��������ε�ͼ�δ�С
			area[0] = leftRect.size.width * leftRect.size.height;
			area[1] = rightRect.size.width * rightRect.size.height;
			double half_height = (leftRect.size.height + rightRect.size.height) / 4;  //�����ߣ���������y�����ƫ��̶ȣ�
			//�ж������������ĵ�ľ���
			distance = Distance(leftRect.center, rightRect.center);
			height = (leftRect.size.height + rightRect.size.height) / 2;      //�����ε�ƽ���߶�

			if (abs(leftRect.angle - rightRect.angle) < 30 && min(area[0], area[1]) * 4 > max(area[0], area[1])
				&& abs(leftRect.center.y - rightRect.center.y) < 1 * half_height && distance < 6 * height && distance > 1.5*height)
			{
				//�õ�װ�װ����ĵ������
				currentCenter.x = (minRects[i].center.x + minRects[j].center.x) / 2;
				currentCenter.y = (minRects[i].center.y + minRects[j].center.y) / 2;
				//�õ��������±ߵ��е�
				leftRect_up.x = minRects[i].center.x;      //�������Ϸ��е�  
				leftRect_up.y = minRects[i].center.y - (minRects[i].size.height) / 2;
				leftRect_down.x = minRects[i].center.x;    //�������·��е�  
				leftRect_down.y = minRects[i].center.y + (minRects[i].size.height) / 2;

				rightRect_up.x = minRects[j].center.x;     //�Ҳ�����Ϸ��е�  
				rightRect_up.y = minRects[j].center.y - (minRects[j].size.height) / 2;
				rightRect_down.x = minRects[j].center.x;   //�Ҳ�����·��е�  
				rightRect_down.y = minRects[j].center.y + (minRects[j].size.height) / 2;
				//�õ��������
				armor_area = distance * height;
				//�õ����νǶ�
				armor_angle = (leftRect.angle + rightRect.angle) / 2;

				cout << "[Point" << num << "] x = " << currentCenter.x - frame.cols / 2 << "    y = " << currentCenter.y - frame.rows / 2 
					<< "    armor_area = " << armor_area << "    armor_angle = " << armor_angle << endl;
				num++;

				//����װ�װ������
				circle(frame, currentCenter, 2, Scalar(0, 0, 255), 3);
				//��������ƹ�����ĺ�����
				circle(frame, leftRect_up, 2, Scalar(0, 255, 0), 2);
				circle(frame, leftRect_down, 2, Scalar(0, 255, 0), 2);
				circle(frame, rightRect_up, 2, Scalar(0, 255, 0), 2);
				circle(frame, rightRect_down, 2, Scalar(0, 255, 0), 2);
				line(frame, leftRect_up, currentCenter, Scalar(255, 255, 255), 1);
				line(frame, leftRect_down, currentCenter, Scalar(255, 255, 255), 1);
				line(frame, rightRect_up, currentCenter, Scalar(255, 255, 255), 1);
				line(frame, rightRect_down, currentCenter, Scalar(255, 255, 255), 1);
				
                //����һ�εĽ���Ա�
				if (lastCenter.x == 0 && lastCenter.y == 0)
				{
					lastCenter = currentCenter;
					lost = 0;
				}
				else
				{
					double difference = Distance(currentCenter, lastCenter);  //Ѱ������ͼ����е���룬������������LostTarget() ����
					if (difference > 300) {
						LostTarget();
						return currentCenter;
					}
				}


			}
			else continue;
			
    
/*
			//�ж��������εĽǶȲ�ֵ��������ֵͬ��ֵ��level
			if (leftRect.angle == rightRect.angle) 
			{ level += 10;}
			else if (abs(leftRect.angle - rightRect.angle) < 5) 
			{ level += 8;}
			else if (abs(leftRect.angle - rightRect.angle) < 10) 
			{ level += 6;}
			else if (abs(leftRect.angle - rightRect.angle) < 30) 
			{ level += 4;}
			else if (abs(leftRect.angle - rightRect.angle) < 90) 
			{ level += 1;}
			else break;

			//�ж��������ε�ͼ�δ�С
			area[0] = leftRect.size.width * leftRect.size.height;
			area[1] = rightRect.size.width * rightRect.size.height;
			if (area[0] == area[1]) 
			{ level += 10;}
			else if (min(area[0], area[1]) * 1.5 > max(area[0], area[1])) 
			{ level += 8;}
			else if (min(area[0], area[1]) * 2 > max(area[0], area[1])) 
			{ level += 6;}
			else if (min(area[0], area[1]) * 3 > max(area[0], area[1])) 
			{ level += 4;}
			else if (min(area[0], area[1]) * 4 > max(area[0], area[1])) 
			{ level += 1;}
			else break;

			//�ж������������ĵ�ˮƽλ�ã�y���꣩
			double half_height = (leftRect.size.height + rightRect.size.height) / 4;  //�����ߣ���������y�����ƫ��̶ȣ�
			if (leftRect.center.y == rightRect.center.y) 
			{ level += 10;}
			else if (abs(leftRect.center.y - rightRect.center.y) < 0.2 * half_height) 
			{ level += 8;}
			else if (abs(leftRect.center.y - rightRect.center.y) < 0.4 * half_height) 
			{ level += 6;}
			else if (abs(leftRect.center.y - rightRect.center.y) < 0.8 * half_height) 
			{ level += 4;}
			else if (abs(leftRect.center.y - rightRect.center.y) < half_height) 
			{ level += 1;}
			else break;

			//�ж������������ĵ�ľ���
			distance = Distance(leftRect.center, rightRect.center);
			height = (leftRect.size.height + rightRect.size.height) / 2;      //�����ε�ƽ���߶�
			if (distance != 0 && distance > height) {
				if (distance < 1.5 * height) 
				{ level += 6;}
				else if (distance < 1.8 * height) 
				{ level += 4;}
				else if (distance < 2.4 * height) 
				{ level += 2;}
				else if (distance < 10 * height)
				{ level += 1;}
				else break;
			}

			//�õ���������µķ��ϳ̶ȣ�levelֵԽ��Խ���ϣ�
			temp[0] = i;
			temp[1] = j;
			temp[2] = level;

			reliability.push_back(temp);
            */
		}	
		
	}
    imshow("frame��ʶ��װ�װ壩", frame);
	minRects.clear();                        //������������!!!!
    return currentCenter;

	/*
	if (reliability.empty()) 
	{
		LostTarget();
		return currentCenter;
	}
	else 
	{
		int maxLevel = 0, index = 0;
		for (int k = 0; k < reliability.size(); ++k)    //ͨ���������������ȡ�����level�Ͷ�Ӧ����������ҵ�����ϵ���������
		{
			if (reliability[k][2] > maxLevel) 
			{
				maxLevel = reliability[k][2];
				index = k;
			}
		}

		//�õ�װ�װ����ĵ������
		currentCenter.x = (minRects[reliability[index][0]].center.x + minRects[reliability[index][1]].center.x) / 2;
		currentCenter.y = (minRects[reliability[index][0]].center.y + minRects[reliability[index][1]].center.y) / 2;

		//�õ��������±ߵ��е�
		leftRect_up.x = minRects[reliability[index][0]].center.x;      //�������Ϸ��е�  
		leftRect_up.y = minRects[reliability[index][0]].center.y - (minRects[reliability[index][0]].size.height)/2;
		leftRect_down.x = minRects[reliability[index][0]].center.x;    //�������·��е�  
		leftRect_down.y = minRects[reliability[index][0]].center.y + (minRects[reliability[index][0]].size.height) / 2;

		rightRect_up.x = minRects[reliability[index][1]].center.x;     //�Ҳ�����Ϸ��е�  
		rightRect_up.y = minRects[reliability[index][1]].center.y - (minRects[reliability[index][1]].size.height) / 2;
		rightRect_down.x = minRects[reliability[index][1]].center.x;   //�Ҳ�����·��е�  
		rightRect_down.y = minRects[reliability[index][1]].center.y + (minRects[reliability[index][1]].size.height) / 2;

		//����һ�εĽ���Ա�
		if (lastCenter.x == 0 && lastCenter.y == 0) 
		{
			lastCenter = currentCenter;
			lost = 0;
		}
		else 
		{
			double difference = Distance(currentCenter, lastCenter);  //Ѱ������ͼ����е���룬������������LostTarget() ����
			if (difference > 300) {
				LostTarget();
				return currentCenter;
			}
		}

		//����װ�װ������
		circle(frame, currentCenter, 2, Scalar(0, 0, 255), 3);
		//��������ƹ�����ĺ�����
		circle(frame, leftRect_up, 2, Scalar(0, 255, 0), 2);
		circle(frame, leftRect_down, 2, Scalar(0, 255, 0), 2);
		circle(frame, rightRect_up, 2, Scalar(0, 255, 0), 2);
		circle(frame, rightRect_down, 2, Scalar(0, 255, 0), 2);
		line(frame, leftRect_up, currentCenter, Scalar(255, 255, 255), 2);
		line(frame, leftRect_down, currentCenter, Scalar(255, 255, 255), 2);
		line(frame, rightRect_up, currentCenter, Scalar(255, 255, 255), 2);
		line(frame, rightRect_down, currentCenter, Scalar(255, 255, 255), 2);


		imshow("frame��ʶ��װ�װ壩", frame);
		return currentCenter;
	}
    */
}

void ArmorDetection::LostTarget()             //���ͼ����Ϊʶ�𵽺��ʵľ��Σ���ô������һ��ͼ��װ�װ�����ĵ㡣������ʱ����Ϊ0
{
	
	lost++;
	if (lost < 2) {
		currentCenter = lastCenter;
	}
	else {
		currentCenter = Point2f(0, 0);
		lastCenter = Point2f(0, 0);
	}
	/*
	currentCenter = Point2f(0, 0);
	lastCenter = Point2f(0, 0);*/
}

double ArmorDetection::Distance(Point2f a, Point2f b)     //����a, b����ľ���
{
	return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

double ArmorDetection::max(double first, double second)   //�Ƚ��������Ĵ�С�����ؽϴ�ֵ
{
	return first > second ? first : second;
}

double ArmorDetection::min(double first, double second)   //�Ƚ��������Ĵ�С�����ؽ�Сֵ
{
	return first < second ? first : second;
}