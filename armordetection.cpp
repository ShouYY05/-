#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/imgproc/types_c.h>   //色彩空间转换
#include <vector>                      //向量
#include <stdlib.h>
#include "armordetection.h"


ArmorDetection::ArmorDetection() = default;

ArmorDetection::ArmorDetection(Mat & input) {
	frame = input;
}

void ArmorDetection::setInputImage(Mat input)                  //导入当前图像
{
	frame = input;
	currentCenter.x = 0;    //两矩形中心坐标
	currentCenter.y = 0;
	leftRect_up.x = 0;      //左侧矩形上方中点  
	leftRect_up.y = 0;
	leftRect_down.x = 0;    //左侧矩形下方中点  
	leftRect_down.y = 0;
	rightRect_up.x = 0;     //右侧矩形上方中点  
	rightRect_up.y = 0;
	rightRect_down.x = 0;   //右侧矩形下方中点  
	rightRect_down.y = 0;

}

//图像预处理：把图像转换为HSV类型，再利用角点找出边缘，得到最小外接矩形，并画出边缘线和图像中心位置。
void ArmorDetection::Pretreatment() {
	Mat input;
	Point p, center;
	vector<vector<Point>> contours;
	vector<Vec4i> hireachy;
	vector<Rect> boundRect(contours.size());
	Point2f vertex[4];                                         //外接矩形的四个顶点

	imshow("src", frame);
	/*
	//创建进度条
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
	// 形态学操作
	morphologyEx(mask, mask, MORPH_OPEN, kernel1, Point(-1, -1));//开操作
	dilate(mask, mask, kernel2, Point(-1, -1), 1);//膨胀
	//轮廓增强
	Canny(mask, mask, 3, 9, 3);
	imshow("mask", mask);
	*/

	cvtColor(frame, hsv, CV_BGR2HSV);                         //把读取的src图像转为HSV图
	vector<Mat> channels;                                     //分成三个频道
	split(frame, channels);
	Mat hue = channels.at(0);
	Mat value = channels.at(2);
	Mat mask(hue.rows, hue.cols, CV_8UC1, Scalar(0, 0, 0));   //建立纯黑图像画板

	for (int i = 0; i < hue.rows; i++)
	{
		for (int j = 0; j < hue.cols; j++)
		{
			int h = hue.at<uchar>(i, j);                      //该点H、V的值
			int v = value.at<uchar>(i, j);
			if (h > 78 && v < 50)                             //筛选蓝色区域为白色值
				mask.at<uchar>(i, j) = 255;
		}
	}

	imshow("二值边缘图", mask);
	findContours(mask, contours, hireachy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	//筛选，去除一部分矩形
	for (int i = 0; i < contours.size(); ++i) 
	{
		RotatedRect minRect = minAreaRect(Mat(contours[i]));  //最小外接矩形

		minRect.points(vertex);                               //外接矩形的四个顶点
		if (minRect.size.width > minRect.size.height)         //交换矩形的宽和高，使得宽＜高
		{
			minRect.angle += 90;
			float t = minRect.size.width;
			minRect.size.width = minRect.size.height;
			minRect.size.height = t;
		}

        //初步筛选满足条件的矩形,并储存到minRects中
		if ((minRect.size.width * 9 > minRect.size.height)       //矩形宽比高
			&& (minRect.size.width * 1 < minRect.size.height)
			&& (minRect.size.width * minRect.size.height < 160) && (minRect.size.width* minRect.size.height > 20) //矩形面积>20, <160
			&& (abs(minRect.angle) < 30) )                        //矩形偏离角
		{
			minRects.push_back(minRect);
		}

        /*//用线圈出识别的最小矩形（蓝色区域） 所画图像，起点，终点，颜色，线宽
		for (int l = 0; l < 4; l++)
		{
			line(frame, vertex[l], vertex[(l + 1) % 4], Scalar(255, 255, 255), 2);   
		}
		//画出图像中心十字
		line(frame, Point(frame.cols / 2 - 15, frame.rows / 2),
			Point(frame.cols / 2 + 15, frame.rows / 2), Scalar(0, 255, 255), 5);
		line(frame, Point(frame.cols / 2, frame.rows / 2 - 15),
			Point(frame.cols / 2, frame.rows / 2 + 15), Scalar(0, 255, 255), 5);
		circle(frame, Point(frame.cols / 2, frame.rows / 2), 4, Scalar(0, 0, 255), -1);
		*/
	}
}


Point2f ArmorDetection::GetArmorCenter() {
	//遍历所有矩形，两两组合
	RotatedRect leftRect, rightRect;  //左右两个矩形
	vector<int*> reliability;
	double area[2], distance, height;
	

	if (minRects.size() < 2)          //只找到一个矩形或者没有矩形
	{
		LostTarget();
		return currentCenter;
	}

	for (int i = 0; i < minRects.size(); ++i) 
	{
		for (int j = i + 1; j < minRects.size(); ++j) 
		{
			int num = 1;    //同一张图上的不同中心点
			int level = 0;
			int temp[3];
			leftRect = minRects[i];        //遍历所有矩形，两两组合
			rightRect = minRects[j];
			//判断两个矩形的图形大小
			area[0] = leftRect.size.width * leftRect.size.height;
			area[1] = rightRect.size.width * rightRect.size.height;
			double half_height = (leftRect.size.height + rightRect.size.height) / 4;  //定义半高（衡量矩形y坐标的偏离程度）
			//判断两个矩形中心点的距离
			distance = Distance(leftRect.center, rightRect.center);
			height = (leftRect.size.height + rightRect.size.height) / 2;      //两矩形的平均高度

			if (abs(leftRect.angle - rightRect.angle) < 30 && min(area[0], area[1]) * 4 > max(area[0], area[1])
				&& abs(leftRect.center.y - rightRect.center.y) < 1 * half_height && distance < 6 * height && distance > 1.5*height)
			{
				//得到装甲板中心点的坐标
				currentCenter.x = (minRects[i].center.x + minRects[j].center.x) / 2;
				currentCenter.y = (minRects[i].center.y + minRects[j].center.y) / 2;
				//得到矩形上下边的中点
				leftRect_up.x = minRects[i].center.x;      //左侧矩形上方中点  
				leftRect_up.y = minRects[i].center.y - (minRects[i].size.height) / 2;
				leftRect_down.x = minRects[i].center.x;    //左侧矩形下方中点  
				leftRect_down.y = minRects[i].center.y + (minRects[i].size.height) / 2;

				rightRect_up.x = minRects[j].center.x;     //右侧矩形上方中点  
				rightRect_up.y = minRects[j].center.y - (minRects[j].size.height) / 2;
				rightRect_down.x = minRects[j].center.x;   //右侧矩形下方中点  
				rightRect_down.y = minRects[j].center.y + (minRects[j].size.height) / 2;
				//得到矩形面积
				armor_area = distance * height;
				//得到矩形角度
				armor_angle = (leftRect.angle + rightRect.angle) / 2;

				cout << "[Point" << num << "] x = " << currentCenter.x - frame.cols / 2 << "    y = " << currentCenter.y - frame.rows / 2 
					<< "    armor_area = " << armor_area << "    armor_angle = " << armor_angle << endl;
				num++;

				//画出装甲板的中心
				circle(frame, currentCenter, 2, Scalar(0, 0, 255), 3);
				//画出两侧灯光的中心和连线
				circle(frame, leftRect_up, 2, Scalar(0, 255, 0), 2);
				circle(frame, leftRect_down, 2, Scalar(0, 255, 0), 2);
				circle(frame, rightRect_up, 2, Scalar(0, 255, 0), 2);
				circle(frame, rightRect_down, 2, Scalar(0, 255, 0), 2);
				line(frame, leftRect_up, currentCenter, Scalar(255, 255, 255), 1);
				line(frame, leftRect_down, currentCenter, Scalar(255, 255, 255), 1);
				line(frame, rightRect_up, currentCenter, Scalar(255, 255, 255), 1);
				line(frame, rightRect_down, currentCenter, Scalar(255, 255, 255), 1);
				
                //与上一次的结果对比
				if (lastCenter.x == 0 && lastCenter.y == 0)
				{
					lastCenter = currentCenter;
					lost = 0;
				}
				else
				{
					double difference = Distance(currentCenter, lastCenter);  //寻找两个图像的中点距离，如果过大，则进入LostTarget() 函数
					if (difference > 300) {
						LostTarget();
						return currentCenter;
					}
				}


			}
			else continue;
			
    
/*
			//判断两个矩形的角度差值，并将不同值赋值给level
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

			//判断两个矩形的图形大小
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

			//判断两个矩形中心的水平位置（y坐标）
			double half_height = (leftRect.size.height + rightRect.size.height) / 4;  //定义半高（衡量矩形y坐标的偏离程度）
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

			//判断两个矩形中心点的距离
			distance = Distance(leftRect.center, rightRect.center);
			height = (leftRect.size.height + rightRect.size.height) / 2;      //两矩形的平均高度
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

			//得到该种组合下的符合程度（level值越大越符合）
			temp[0] = i;
			temp[1] = j;
			temp[2] = level;

			reliability.push_back(temp);
            */
		}	
		
	}
    imshow("frame（识别装甲板）", frame);
	minRects.clear();                        //清除储存的数据!!!!
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
		for (int k = 0; k < reliability.size(); ++k)    //通过遍历所有情况，取得最大level和对应的组合数，找到最符合的两个矩形
		{
			if (reliability[k][2] > maxLevel) 
			{
				maxLevel = reliability[k][2];
				index = k;
			}
		}

		//得到装甲板中心点的坐标
		currentCenter.x = (minRects[reliability[index][0]].center.x + minRects[reliability[index][1]].center.x) / 2;
		currentCenter.y = (minRects[reliability[index][0]].center.y + minRects[reliability[index][1]].center.y) / 2;

		//得到矩形上下边的中点
		leftRect_up.x = minRects[reliability[index][0]].center.x;      //左侧矩形上方中点  
		leftRect_up.y = minRects[reliability[index][0]].center.y - (minRects[reliability[index][0]].size.height)/2;
		leftRect_down.x = minRects[reliability[index][0]].center.x;    //左侧矩形下方中点  
		leftRect_down.y = minRects[reliability[index][0]].center.y + (minRects[reliability[index][0]].size.height) / 2;

		rightRect_up.x = minRects[reliability[index][1]].center.x;     //右侧矩形上方中点  
		rightRect_up.y = minRects[reliability[index][1]].center.y - (minRects[reliability[index][1]].size.height) / 2;
		rightRect_down.x = minRects[reliability[index][1]].center.x;   //右侧矩形下方中点  
		rightRect_down.y = minRects[reliability[index][1]].center.y + (minRects[reliability[index][1]].size.height) / 2;

		//与上一次的结果对比
		if (lastCenter.x == 0 && lastCenter.y == 0) 
		{
			lastCenter = currentCenter;
			lost = 0;
		}
		else 
		{
			double difference = Distance(currentCenter, lastCenter);  //寻找两个图像的中点距离，如果过大，则进入LostTarget() 函数
			if (difference > 300) {
				LostTarget();
				return currentCenter;
			}
		}

		//画出装甲板的中心
		circle(frame, currentCenter, 2, Scalar(0, 0, 255), 3);
		//画出两侧灯光的中心和连线
		circle(frame, leftRect_up, 2, Scalar(0, 255, 0), 2);
		circle(frame, leftRect_down, 2, Scalar(0, 255, 0), 2);
		circle(frame, rightRect_up, 2, Scalar(0, 255, 0), 2);
		circle(frame, rightRect_down, 2, Scalar(0, 255, 0), 2);
		line(frame, leftRect_up, currentCenter, Scalar(255, 255, 255), 2);
		line(frame, leftRect_down, currentCenter, Scalar(255, 255, 255), 2);
		line(frame, rightRect_up, currentCenter, Scalar(255, 255, 255), 2);
		line(frame, rightRect_down, currentCenter, Scalar(255, 255, 255), 2);


		imshow("frame（识别装甲板）", frame);
		return currentCenter;
	}
    */
}

void ArmorDetection::LostTarget()             //如果图像中为识别到合适的矩形，那么返回上一张图像装甲板的中心点。差距过大时重置为0
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

double ArmorDetection::Distance(Point2f a, Point2f b)     //计算a, b两点的距离
{
	return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

double ArmorDetection::max(double first, double second)   //比较两个数的大小，返回较大值
{
	return first > second ? first : second;
}

double ArmorDetection::min(double first, double second)   //比较两个数的大小，返回较小值
{
	return first < second ? first : second;
}