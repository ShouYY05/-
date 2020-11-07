#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/imgproc/types_c.h>   //色彩空间转换
#include <vector>                      //向量
#include <stdlib.h>
#include "armordetection.h"
#include "LightBar.h"
#include "armor.h"

ArmorDetection::ArmorDetection() = default;



//导入当前图像
void ArmorDetection::setInputImage(Mat input)                  
{
	frame = input;
}

/**
 * @brief 图像预处理：把图像转换为HSV类型，再利用角点找出边缘，得到最小外接矩形，
 *        并画出边缘线和图像中心位置。将符合要求的矩形存储在lights里面
 */
void ArmorDetection::Pretreatment(vector<LightBar> lights) {
	Mat input;
	Point p, center;
	vector<vector<Point>> contours;
	vector<Vec4i> hireachy;
	vector<Rect> boundRect(contours.size());
	Point2f vertex[4];       //外接矩形的四个顶点
	
	imshow("src", frame);

	cvtColor(frame, hsv, CV_BGR2HSV);                      
	vector<Mat> channels;                                 
	split(frame, channels);
	Mat hue = channels.at(0);
	Mat value = channels.at(2);
	Mat mask(hue.rows, hue.cols, CV_8UC1, Scalar(0, 0, 0));  

	for (int i = 0; i < hue.rows; i++)
	{
		for (int j = 0; j < hue.cols; j++)
		{
			int h = hue.at<uchar>(i, j);                  
			int v = value.at<uchar>(i, j);
			if (h > 78 && v < 50)                //筛选蓝色区域为白色值
				mask.at<uchar>(i, j) = 255;
		}
	}
	imshow("二值边缘图", mask);
	findContours(mask, contours, hireachy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	//筛选，去除一部分矩形
	for (int i = 0; i < contours.size(); ++i) 
	{
		RotatedRect minRect = minAreaRect(Mat(contours[i]));  //最小外接矩形

		if (minRect.size.width > minRect.size.height)         //交换矩形的宽和高，使得宽＜高
		{
			minRect.angle += 90;
			float t = minRect.size.width;
			minRect.size.width = minRect.size.height;
			minRect.size.height = t;
		}
		

        //初步筛选满足条件的矩形,并储存到lights中
		if ((minRect.size.width * 9 > minRect.size.height)       //矩形宽比高
			&& (minRect.size.width * 1 < minRect.size.height)
			&& (minRect.size.width * minRect.size.height < 160) && (minRect.size.width* minRect.size.height > 20) //矩形面积>20, <160
			&& (abs(minRect.angle) < 30) )                        //矩形偏离角
		{
			minRect.points(vertex);                               //外接矩形的四个顶点
			LightBar light = LightBar(minRect, contourArea(contours[i]));
			light.rect = minRect;
			lights.push_back(light);

			for (int l = 0; l < 4; l++)
			{
				line(frame, vertex[l], vertex[(l + 1) % 4], Scalar(255, 255, 255), 2);   
			}
		}  
	}
}



/**
 * @brief 遍历匹配矩形，存到armor中，并在图像上画出标记点
 * @param 光条类组成的向量，装甲板类组成的向量
 */
void ArmorDetection::GetArmor(vector<LightBar> lights, vector<Armor> armors) 
{
	//遍历所有矩形，两两组合
	
	//左右两个灯棒
	LightBar leftRect, rightRect;  
	vector<int*> reliability;
	double area[2], distance, height;
	
	//遍历所有矩形，两两组合
	for (int i = 0; i < lights.size(); ++i)
	{
		cout << "start";
		for (int j = i + 1; j < lights.size(); ++j)
		{
			int num = 0;    //同一张图上的不同装甲板
			int level = 0;
			int temp[3];
			leftRect = lights[i];
			leftRect = lights[j];
			double half_height = (leftRect.height + rightRect.height) / 4;  //定义半高（衡量矩形y坐标的偏离程度）height 
			//判断两个矩形中心点的距离
			distance = Distance(leftRect.center, rightRect.center);
			height = (leftRect.height + rightRect.height) / 2;      //两矩形的平均高度 > ma
			

			if (abs(leftRect.angle - rightRect.angle) < 30 && min(leftRect.area, rightRect.area) * 2 > max(leftRect.area, rightRect.area)
				&& abs(leftRect.height - rightRect.height) < 2 * half_height && distance < 6 * height && distance > 1.5*height)
			{
				armors.push_back(Armor(leftRect, leftRect));
				
				cout << "[Point" << num << "] x = " << armors[num].currentCenter.x << "    y = " << armors[num].currentCenter.y
					<< "    armor_area = " << armors[num].armor_area << "    armor_angle = " << armors[num].armor_angle << endl;

				//画出装甲板的中心
				circle(frame, armors[num].currentCenter, 2, Scalar(0, 0, 255), 3);   //红
				//画出两侧灯光的中心和连线
				circle(frame, armors[num].leftRect_up, 2, Scalar(0, 255, 0), 2);     //绿
				circle(frame, armors[num].leftRect_down, 2, Scalar(255, 255, 0), 2); //青
				circle(frame, armors[num].rightRect_up, 2, Scalar(0, 255, 255), 2);  //黄
				circle(frame, armors[num].rightRect_down, 2, Scalar(255, 0, 0), 2);  //蓝
				line(frame, armors[num].leftRect_up, currentCenter, Scalar(255, 255, 255), 1);
				line(frame, armors[num].leftRect_down, currentCenter, Scalar(255, 255, 255), 1);
				line(frame, armors[num].rightRect_up, currentCenter, Scalar(255, 255, 255), 1);
				line(frame, armors[num].rightRect_down, currentCenter, Scalar(255, 255, 255), 1);
				
                num++;
			}
			else cout << "end" << endl;
				continue;

			
    
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

	lights.clear();                        //清除储存的数据!!!!
	armors.clear();

}

/**
 * @brief 如果图像中为识别到合适的矩形，那么返回上一张图像装甲板的中心点。差距过大时重置为0
 */
void ArmorDetection::LostTarget()         
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

//计算a, b两点的距离
double ArmorDetection::Distance(Point2f a, Point2f b)     
{
	return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

//比较两个数的大小，返回较大值
double ArmorDetection::max(double first, double second)   
{
	return first > second ? first : second;
}

//比较两个数的大小，返回较小值
double ArmorDetection::min(double first, double second)   
{
	return first < second ? first : second;
}

//将相机坐标系二维坐标转变为三维坐标
void transform(Armor armor)
{
	//自己定义世界坐标系坐标   
	vector<Point3d> objP;  
	vector<Point2d> points;       //相机坐标系的二维坐标
	double q[4];
	Mat rotM, rotT,rvec,tvec;
	
	objP.clear();
	objP.push_back(Point3f(0,0,0));            
	objP.push_back(Point3f(0,26.5, 0));
	objP.push_back(Point3f(67.5,26.5, 0));
	objP.push_back(Point3f(67.5,0, 0));
	
	points.clear();
	points.push_back(armor.leftRect_down);
	points.push_back(armor.leftRect_up);
	points.push_back(armor.rightRect_up);
	points.push_back(armor.rightRect_down);
	
	//使用pnp解算求出相机到世界坐标系的旋转矩阵和平移矩阵       
	solvePnP(objP, points, camera_matrix, distortion_coefficients, rvec, tvec);
	//将旋转向量变换成旋转矩阵
	Rodrigues(rvec, rotM);  
	
	getQuaternion(rotM, q);


}

//计算变换矩阵
void getQuaternion(Mat R, double Q[])
{
	double trace = R.at<double>(0,0) + R.at<double>(1,1) + R.at<double>(2,2);
	if (trace > 0.0) 
	{
		double s = sqrt(trace + 1.0);
		Q[3] = (s * 0.5);
		s = 0.5 / s;
		Q[0] = ((R.at<double>(2,1) - R.at<double>(1,2)) * s);
		Q[1] = ((R.at<double>(0,2) - R.at<double>(2,0)) * s);
		Q[2] = ((R.at<double>(1,0) - R.at<double>(0,1)) * s);
	} 
	else 
	{
		int i = R.at<double>(0,0) < R.at<double>(1,1) ? (R.at<double>(1,1) < R.at<double>(2,2) ? 2 : 1) : (R.at<double>(0,0) < R.at<double>(2,2) ? 2 : 0); 
		int j = (i + 1) % 3; 
		int k = (i + 2) % 3;
		
		double s = sqrt(R.at<double>(i, i) - R.at<double>(j,j) - R.at<double>(k,k) + 1.0);
		Q[i] = s * 0.5;
		s = 0.5 / s;

		Q[3] = (R.at<double>(k,j) - R.at<double>(j,k)) * s;
		Q[j] = (R.at<double>(j,i) + R.at<double>(i,j)) * s;
		Q[k] = (R.at<double>(k,i) + R.at<double>(i,k)) * s;
		}
}
