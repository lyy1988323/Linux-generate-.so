#ifndef INCLUDE_H
#define INCLUDE_H
#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

// 云台、相机参数输入
typedef struct ptz_camera_in
{
	int zoom0;                 //第1步放大及调焦值
	int focus0;
	int zoom1;                 //第2步放大及调焦值
	int focus1;
	int zoom2;                 //第3步放大及调焦值
	int focus2;
	float H1;                  //第1步云台水平角度
	float V1;                  //云台垂直角度
	float deltaH1;             //第2步云台水平角度 = H1 + deltaH1
	float deltaV1;             //第2步云台垂直角度 = V1 + deltaV1
	float deltaH2;             //第3步云台水平角度 = H1 + deltaH2
	float deltaV2;             //第3步云台垂直角度 = V1 + deltaV2
} ptz_camera_i;

// 云台、相机参数输出
typedef struct ptz_camera_out
{
	int zoom;        //相机变倍值
	int focus;	     //相机聚焦值
	float H;	     //云台水平角度
	float V;	     //云台垂直角度

	// 区域聚焦				 
	int x1; //左上角点横坐标
	int y1; //左上角点纵坐标
	int x2; //右下角点横坐标
	int y2; //右下角点纵坐标
	int RegionFocus_En; //区域聚焦使能位

	// 以下参数未使用到					
	char WDR;	             //相机宽动态功能   0-不启用，1-启用，2-自动
	char WDRLevel1;          //宽动态等级，0-F
	char BackLight;		     //相机背光补偿功能，0-off，1-UP，2-DOWN，3-LEFT，4-RIGHT，5-MIDDLE，6-自定义，10-开，11-自动，12-多区域背光补偿
	int iris;	             //相机光圈值
	int shutter;	         //相机快门值
	char ExposureMode;       //相机曝光模式，0-手动模式，1-自动曝光，2-光圈优先，3-快门优先，4-增益优先
	string ImagePath;        //相机拍照保存的路径
} ptz_camera_o;

//结果输出
typedef struct roiResult
{
	int Category;      //识别区域类型
	int Roi_num;       //识别区域数目
	float Value[2];    //识别区域数值
	int Status[50];    //识别区域状态(0或1)
	int Float0_int1;   //读数类型
}roiResult_s;


// 脱机算法纠偏函数
//[in]szScriptPath:脚本路径
//[in]szPicPath : 图片路径
//[in]Zoom : 第一步相机倍率
//[in|out]pX : 水平方向偏差值
//[in|out]pY : 垂直方向偏差值
// 返回值
// 成功 0
// 失败 -1
int AdjustAngle(char *szScriptPath, char *szPicPath, int Zoom, float *pX, float *pY);


// 功能描述：脱机识别算法接口
// 参数描述：szScriptPath:脚本路径
//           szPicPath: 图片路径
//           bestImg_Path 结果图路径
//           Step 步数次数
//           ptz_camera_i 云台、相机参数输入结构体
//           ptz_camera_o 云台、相机参数输出结构体
//           roiResult_s  结果值输出
// 返回值： -1或0或1 （-1识别失败，0再次进入算法 1识别成功）
//int machine_vision_process_v6_Liunx(char* temp_Path, char* srcImg_Data, char* bestImg_Path, int Step,
//	ptz_camera_i*Ptz_Camera_i, ptz_camera_o*Ptz_Camera_o, roiResult_s*results);


#endif
