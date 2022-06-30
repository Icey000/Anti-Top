#include "Monocular.h"
#include <opencv2/opencv.hpp>

/**
 * @brief 初始化相机相机内参
 */

Monocular::Monocular(){
    this->CAMERA_MATRIX = ( Mat_<double>(3,3)  << fx,0,u0,0,fy,v0,0,0,1);
    this->DISTORTION_COEFF = ( Mat_<double>(5,1)  << k1,k2,p1,p2) ;
    this->rVec = Mat::zeros(3, 1, CV_64FC1);
    this->tVec = Mat::zeros(3, 1, CV_64FC1);
}
/*
Monocular::Monocular(){
    Mat cameraMatrix;
    Mat distCoeffs;
    cv::FileStorage fs("/home/bbtx2/standard_rm/src/cv/include/Position/cameraParams.xml", FileStorage::READ);
    if(!fs.isOpened()) cout<<"Error: File is not opened!"<<endl;
    fs["camera-matrix"] >> cameraMatrix;
    fs["distortion"] >> distCoeffs;
    this->CAMERA_MATRIX = ( cameraMatrix );
    this->DISTORTION_COEFF = ( distCoeffs );
    this->rVec = Mat::zeros(3, 1, CV_64FC1);
    this->tVec = Mat::zeros(3, 1, CV_64FC1);
}
*/
/**
 * @brief 设置装甲板的实际宽和高
 * 
 * @param armorType BIG or SMALL
 * @param width 装甲板的实际宽度
 * @param height 装甲板的实际高度
 */
void Monocular::setArmorSize(ArmorType armorType){
    double half_x;
	double half_y;
	switch (armorType){

        case SMALL_ARMOR:
	      half_x = SmallArmor_width/2.0;
            half_y = SmallArmor_height/2.0;
            SMALL_ARMOR_POINTS_3D.push_back(Point3f(-half_x, half_y, 0));   //tl top left
            SMALL_ARMOR_POINTS_3D.push_back(Point3f(half_x, half_y, 0));    //tr top right
            SMALL_ARMOR_POINTS_3D.push_back(Point3f(half_x, -half_y, 0));   //br below right
            SMALL_ARMOR_POINTS_3D.push_back(Point3f(-half_x, -half_y, 0));  //bl below left
            break;

        case BIG_ARMOR:

	     half_x = BigArmor_width/2.0;
            half_y = BigArmor_height/2.0;
            BIG_ARMOR_POINTS_3D.push_back(Point3f(-half_x, half_y, 0));   //tl top left
            BIG_ARMOR_POINTS_3D.push_back(Point3f(half_x, half_y, 0));    //tr top right
            BIG_ARMOR_POINTS_3D.push_back(Point3f(half_x, -half_y, 0));   //br below right
            BIG_ARMOR_POINTS_3D.push_back(Point3f(-half_x, -half_y, 0));  //bl below left
            break;
        
        default: 
            break;
	}
}

/**
 * @brief 设置装甲板所对应的图像二维坐标
 * 
 * @param contour 装甲板图像的四个轮廓点：left_up, right_up, left_down, right_down
 * @param center  装甲板图像的中心坐标
 * @param type    装甲板的类型
 */
void Monocular::setTarget2D( vector<Point2f> contour, Point2f center, ArmorType type ){
    this->targetContour = contour;
    this->targetCenter = center;
    this->targetType = type;
}

/**
 * @brief 定位，计算装甲板在相机坐标系下的三维坐标
 */
void Monocular::getArmorCoordinate(){
	/**cout<<"@@@@@@@@@@@@@@@ targetContour's size: "<<targetContour.size()<<" @@@@@@@@@@@@@@@"<<endl;
	cout<<targetContour<<endl;
	cout<<"small: "<<SMALL_ARMOR_POINTS_3D<<endl;
	cout<<"big: "<<BIG_ARMOR_POINTS_3D<<endl;	
	cout<<"targetArmorType:"<<targetType<<endl;
	*/
    if(!CAMERA_MATRIX.empty() && !DISTORTION_COEFF.empty()){ 
		switch (targetType){
		    case SMALL_ARMOR:
		        solvePnP(SMALL_ARMOR_POINTS_3D, targetContour, CAMERA_MATRIX, DISTORTION_COEFF, rVec, tVec, false, SOLVEPNP_ITERATIVE); break;
		    case BIG_ARMOR:
		        solvePnP(BIG_ARMOR_POINTS_3D, targetContour, CAMERA_MATRIX, DISTORTION_COEFF, rVec, tVec, false, SOLVEPNP_ITERATIVE); break;
		    default:
				cout<<"----------------------Error: No Matched Armor Type!----------------------"<<endl;
				break;
		}

		this->x_pos = tVec.at<double>(0, 0);
		this->y_pos = tVec.at<double>(1, 0);
		this->z_pos = tVec.at<double>(2, 0);
		this->distance = sqrt(x_pos * x_pos + y_pos * y_pos + z_pos * z_pos);
		if(this->distance < 3000)	this->shoot = 1;
		else	this->shoot = 0;
	}else{
		cout<<"Warnning: CAMERA_MATRIX or DISTORTION_COEFF is null!"<<endl;
	}
    //清空本次记录的坐标信息，为下次定位留出空间
    this->clearCoordinateInfo();
}

/**
 * @brief 清空记录的所有坐标信息：2D, 3D
 */
void Monocular::clearCoordinateInfo(){
    //2D
    this->targetContour.clear();
    
    //3D
    this->SMALL_ARMOR_POINTS_3D.clear();
    this->BIG_ARMOR_POINTS_3D.clear();
}

/**
 * @brief 显示一些定位信息
 */
void Monocular::showDebugInfo(int showCordin, int showDist ){
    
    if(showCordin == 1){
        cout<<"x: "<<this->x_pos<<endl;
        cout<<"y: "<<this->y_pos<<endl;
        cout<<"z: "<<this->z_pos<<endl;
    }

    if(showDist == 1){
        cout<<"distance: "<<this->distance<<endl;
    } 
}



