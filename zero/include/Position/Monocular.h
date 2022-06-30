#ifndef MONOCULAR_H
#define MONOCULAR_H

#include "../General/General.h"

class Monocular{

public:
    //Camera params
	Mat CAMERA_MATRIX;    //IntrinsicMatrix		  fx,fy,cx,cy
	Mat DISTORTION_COEFF; //DistortionCoefficients k1,k2,p1,p2

	//Object points in world coordinate
	vector<Point3f> SMALL_ARMOR_POINTS_3D;
	vector<Point3f> BIG_ARMOR_POINTS_3D;

	//Targets
	vector<Point2f> targetContour;
	Point2f targetCenter;
	ArmorType targetType;

	// calculated by solvePnP
	// s[R|t]=s'  s->world coordinate;s`->camera coordinate
	Mat rVec;     //rot rotation between camera and target center
	Mat tVec;     //trans tanslation between camera and target center

    //results
    double x_pos, y_pos, z_pos; //装甲板坐标
    double distance;    //相机距装甲板的距离
    int shoot;	//射击标识

public:

    /**
     * @brief 初始化相机相机内参
     */
    Monocular();

    /**
     * @brief 设置装甲板的实际宽和高
     * 
     * @param armorType BIG or SMALL
     * @param width 装甲板的实际宽度
     * @param height 装甲板的实际高度
     */
    void setArmorSize(ArmorType armorType);

    /**
     * @brief 设置装甲板所对应的图像二维坐标
     * 
     * @param contour 装甲板图像的四个轮廓点：left_up, right_up, left_down, right_down
     * @param center  装甲板图像的中心坐标
     * @param type    装甲板的类型
     */
    void setTarget2D( vector<Point2f> contour, Point2f center, ArmorType type );


    /**
     * @brief 定位，计算装甲板在相机坐标系下的三维坐标
     */
    void getArmorCoordinate();

    /**
     * @brief 清空记录的所有坐标信息：2D, 3D
     */
    void clearCoordinateInfo();

    /**
     * @brief 显示一些定位信息
     */
    void showDebugInfo(int showCordin, int showDist );

};


#endif
