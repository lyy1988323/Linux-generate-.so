#ifndef INCLUDE_H
#define INCLUDE_H
#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

// ��̨�������������
typedef struct ptz_camera_in
{
	int zoom0;                 //��1���Ŵ󼰵���ֵ
	int focus0;
	int zoom1;                 //��2���Ŵ󼰵���ֵ
	int focus1;
	int zoom2;                 //��3���Ŵ󼰵���ֵ
	int focus2;
	float H1;                  //��1����̨ˮƽ�Ƕ�
	float V1;                  //��̨��ֱ�Ƕ�
	float deltaH1;             //��2����̨ˮƽ�Ƕ� = H1 + deltaH1
	float deltaV1;             //��2����̨��ֱ�Ƕ� = V1 + deltaV1
	float deltaH2;             //��3����̨ˮƽ�Ƕ� = H1 + deltaH2
	float deltaV2;             //��3����̨��ֱ�Ƕ� = V1 + deltaV2
} ptz_camera_i;

// ��̨������������
typedef struct ptz_camera_out
{
	int zoom;        //����䱶ֵ
	int focus;	     //����۽�ֵ
	float H;	     //��̨ˮƽ�Ƕ�
	float V;	     //��̨��ֱ�Ƕ�

	// ����۽�				 
	int x1; //���Ͻǵ������
	int y1; //���Ͻǵ�������
	int x2; //���½ǵ������
	int y2; //���½ǵ�������
	int RegionFocus_En; //����۽�ʹ��λ

	// ���²���δʹ�õ�					
	char WDR;	             //�����̬����   0-�����ã�1-���ã�2-�Զ�
	char WDRLevel1;          //��̬�ȼ���0-F
	char BackLight;		     //������ⲹ�����ܣ�0-off��1-UP��2-DOWN��3-LEFT��4-RIGHT��5-MIDDLE��6-�Զ��壬10-����11-�Զ���12-�����򱳹ⲹ��
	int iris;	             //�����Ȧֵ
	int shutter;	         //�������ֵ
	char ExposureMode;       //����ع�ģʽ��0-�ֶ�ģʽ��1-�Զ��ع⣬2-��Ȧ���ȣ�3-�������ȣ�4-��������
	string ImagePath;        //������ձ����·��
} ptz_camera_o;

//������
typedef struct roiResult
{
	int Category;      //ʶ����������
	int Roi_num;       //ʶ��������Ŀ
	float Value[2];    //ʶ��������ֵ
	int Status[50];    //ʶ������״̬(0��1)
	int Float0_int1;   //��������
}roiResult_s;


// �ѻ��㷨��ƫ����
//[in]szScriptPath:�ű�·��
//[in]szPicPath : ͼƬ·��
//[in]Zoom : ��һ���������
//[in|out]pX : ˮƽ����ƫ��ֵ
//[in|out]pY : ��ֱ����ƫ��ֵ
// ����ֵ
// �ɹ� 0
// ʧ�� -1
int AdjustAngle(char *szScriptPath, char *szPicPath, int Zoom, float *pX, float *pY);


// �����������ѻ�ʶ���㷨�ӿ�
// ����������szScriptPath:�ű�·��
//           szPicPath: ͼƬ·��
//           bestImg_Path ���ͼ·��
//           Step ��������
//           ptz_camera_i ��̨�������������ṹ��
//           ptz_camera_o ��̨�������������ṹ��
//           roiResult_s  ���ֵ���
// ����ֵ�� -1��0��1 ��-1ʶ��ʧ�ܣ�0�ٴν����㷨 1ʶ��ɹ���
//int machine_vision_process_v6_Liunx(char* temp_Path, char* srcImg_Data, char* bestImg_Path, int Step,
//	ptz_camera_i*Ptz_Camera_i, ptz_camera_o*Ptz_Camera_o, roiResult_s*results);


#endif
