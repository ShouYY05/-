#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "armordetection.h"

using namespace cv;
using namespace std;

ArmorDetection* armor = new ArmorDetection();
Point2f center;

int main()
{
	//fps变量
	double t = (double)getTickCount();
	double fps;
	char string[10];
	char string2[10];
	Mat frame;

VideoCapture capture("装甲板.mp4");         //导入视频
	if (!capture.isOpened())                //报错
	{
		printf("无法打开相机...\n");
		return -1;
	}
	
	//namedWindow("src", 1);
	//namedWindow("frame", 1);
	//namedWindow("mask", 1);
	//namedWindow("Control", 1);

	while (capture.read(frame))           //读取当前帧
	{
		armor->setInputImage(frame);      //导入当前图像
		armor->Pretreatment();            //图像预处理
		center = armor->GetArmorCenter();

		//cout << "[INFO] x = " << center.x - frame.cols / 2 << "    y = " << center.y - frame.rows / 2 << endl;

		//计算fps
		double dt = ((double)getTickCount() - t) / (double)getTickFrequency();
		fps = 1.0 / dt;
		t = (double)getTickCount();
		cout << "fps=" << fps << endl;    //在控制台上输出
		sprintf_s(string, "%.2f", fps);
		std::string fpsString("FPS:");
		fpsString += string;
		putText(frame, fpsString, Point(5, 20), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255));

		if (waitKey(1) > 0) break;
	}
	capture.release();//释放视频内存
	waitKey(1500);
	return 0;
}