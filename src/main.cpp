#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "armordetection.h"
#include "armor.h"

using namespace cv;
using namespace std;

ArmorDetection* armordetection = new ArmorDetection();
Point2f center;

int main()
{
	//fps����
	double t = (double)getTickCount();
	double fps;
	char string[10];
	char string2[10];
	Mat frame;

VideoCapture capture("װ�װ�1.mp4");         //������Ƶ
	if (!capture.isOpened())                //����
	{
		printf("�޷������...\n");
		return -1;
	}
	
	//namedWindow("src", 1);
	//namedWindow("frame", 1);
	//namedWindow("mask", 1);
	//namedWindow("Control", 1);

	while (capture.read(frame))           //��ȡ��ǰ֡
	{
		vector<LightBar> lights;                                  //�洢�ư�����
		vector<Armor> armors;
		armordetection->setInputImage(frame);      //���뵱ǰͼ��
		armordetection->Pretreatment(lights);            //ͼ��Ԥ����
		armordetection->GetArmor(lights, armors);

		//cout << "[INFO] x = " << center.x - frame.cols / 2 << "    y = " << center.y - frame.rows / 2 << endl;

		//����fps
		double dt = ((double)getTickCount() - t) / (double)getTickFrequency();
		fps = 1.0 / dt;
		t = (double)getTickCount();
		cout << "fps=" << fps << endl;    //�ڿ���̨�����
		sprintf_s(string, "%.2f", fps);
		std::string fpsString("FPS:");
		fpsString += string;
		putText(frame, fpsString, Point(5, 20), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255));

		if (waitKey(30) == 27) break;
	}
	capture.release();//�ͷ���Ƶ�ڴ�
	waitKey(1500);
	return 0;
}