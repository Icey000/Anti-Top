#ifndef GENERAL_H
#define GENERAL_H

#include<opencv2/opencv.hpp>
#include<iostream>
#include<math.h>

using namespace cv;
using namespace std;
using namespace cv::ml;
enum EnemyColor{
	RED = 0,
	BLUE = 1
};

enum ArmorType{
	SMALL_ARMOR = 0,
	BIG_ARMOR = 1
};

/**
 * @brief 1-BIG_ARMOR, 3,4,5-SMALL_ARMOR
 */
enum ArmorNum{
	ONE = 1,
	THREE = 3,
	FOUR = 4,
	FIVE = 5
};

/**
 * @brief 识别追踪状态
 */
enum DetectorState
{
	LIGHTS_NOT_FOUND = 0,
	LIGHTS_FOUND = 1,
	ARMOR_NOT_FOUND = 2,
	ARMOR_FOUND = 3
};


/******************************************
 * 相机的内参( 设置为全局变量，便于更改相机内参 )
 *****************************************/
/**
 * @brief 相机的内参矩阵
 */
double fx = 2323.5; 
double fy = 2320.5; 
double u0 = 725.5111; 
double v0 = 572.2628;

/**
 * @brief 相机的畸变参数
 */
//1).径向畸变：（Radial Distortion）
double k1 = -0.1060; 
double k2 = 0.4074; 
double k3 = 0.0;
//2).切向畸变：（Tangential Distortion）
double p1 = 0; 
double p2 = 0;

/********************************
 *装甲板的实际宽和高(mm)
 ********************************/
int BigArmor_width  = 225;
int BigArmor_height = 60;

int SmallArmor_width  = 135;
int SmallArmor_height = 60;


#endif
