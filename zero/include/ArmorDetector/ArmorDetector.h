#ifndef ARMOR_H
#define ARMOR_H
#include <cv/yapi.h>
#include "../General/General.h"

/**********************************************
 *@brief: 装甲板相关的各项参数
 **********************************************/ 
struct ArmorParam{

	float min_area;					// min area of light bar 灯条允许的最小面积
	float max_angle;				//max angle of light bar 灯条允许的最大偏角

	float max_angle_diff; 			//max angle difference between two light bars 两个灯条之间允许的最大角度差
	float max_lengthDiff_ratio; 	//max length ratio difference between two light bars 两个灯条之间允许的最大长度差比值
	float max_deviation_angle; 		//max deviation angle 两灯条最大错位角

	float max_y_diff_ratio;  		//max y 
	float max_x_diff_ratio;  		//max x


	//default values  给各参数设定默认值
	ArmorParam(){
	min_area = 10;
	max_angle = 45;

	max_angle_diff = 6;
	max_lengthDiff_ratio = 0.5;
	max_deviation_angle = 50;

	max_y_diff_ratio = 0.5;
	max_x_diff_ratio = 4.5;
	}
};

ArmorParam armorParam;


/************************************************
 * @brief 灯条类
 ************************************************/
class LightBar{

public:
	RotatedRect lightRect;	        //最小外接旋转矩
	Point2f center;			//灯条中心点坐标
	float length;			//灯条长度
	float angle;			//灯条的旋转角度

public:
	LightBar();
	LightBar( const RotatedRect &lightRect );
	~LightBar();

};


/************************************************
 * @brief 装甲板类
 ************************************************/
class ArmorBox{

public:
	LightBar l_light, r_light;  	//装甲板的左右灯条
	int l_index, r_index; 			//左右灯条的下标(默认为-1，仅作为ArmorDetector类成员时生效) 
	int armorNum;  					//装甲板上的数字（用SVM识别得到）
	vector<Point2f> armorVertices;  //装甲板的四个顶点 bl->tl->tr->br   左下 左上 右上 右下
	ArmorType type; 				//the type of armor
	Point2f center;					// center point(crossPoint) of armor 装甲板中心
	Rect armorRect;  				//armorRect for roi 装甲板的矩形获取roi用
	int armorArea;					//Area of armorBox
	float armorAngle;				//armor angle(mean of lightBars) 装甲板角度(灯条角度的平均值)
	Mat armorImg;					//image of armor set by getArmorImg() from ArmorNumClassifier() 装甲板的图片（透射变换获得）

public:

	ArmorBox();
	/**
 	 *@brief: Parametrical constructor of armorBox 装甲板有参构造函数
	 *@param: two LightBar  左右两个灯条
 	 */
	ArmorBox(const LightBar& l_light, const LightBar& r_light, int shriTimes);
	~ArmorBox();

	// angle difference: the angle difference of left and right lights 装甲板左右灯条角度差
	float getAngleDiff() const;

	// deviation angle : the horizon angle of the line of centers of lights 灯条错位度角(两灯条中心连线与水平线夹角) 
	float getDeviationAngle() const;
	
	// dislocation judge X: r-l light center distance ration on the X-axis 灯条位置差距 两灯条中心x方向差距比值
	float getDislocationX() const;
	
	// dislocation judge Y:  r-l light center distance ration on the Y-axis 灯条位置差距 两灯条中心Y方向差距比值
	float getDislocationY() const;

	// length difference ration: the length difference ration r-l lights 左右灯条长度差比值
	float getLengthRation() const;
	
	// an integrative function to judge whether this armor is suitable or not
	bool isSuitableArmor() const;

	//计算装甲板的面积
	void calcuArmorArea();
	
	void reset2DPoints( int shriTimes );

	void recover2DPoints( int shriTimes );
};

/**
 * @brief: use warpPerspective to get armorImg and SVM to recognize armorImg 利用透射变换截取装甲板图片（SVM模型大小），并利用SVM来识别装甲板数字
 */
class ArmorNumClassifier
{
public:
	Ptr<SVM> svm;  //svm model svm模型
	Mat p;		//preRecoginze matrix for svm 载入到SVM中识别的矩阵
	Size armorImgSize; //svm model training dataset size SVM模型的识别图片大小（训练集的图片大小）

	Mat warpPerspective_src; //warpPerspective srcImage  透射变换的原图
	Mat warpPerspective_dst; //warpPerspective dstImage   透射变换生成的目标图
	Mat warpPerspective_mat; //warpPerspective transform matrix 透射变换的变换矩阵
	Point2f srcPoints[4];   //warpPerspective srcPoints		透射变换的原图上的目标点 tl->tr->br->bl  左上 右上 右下 左下
	Point2f dstPoints[4];	//warpPerspective dstPoints     透射变换的目标图中的点   tl->tr->br->bl  左上 右上 右下 左下
public:
	ArmorNumClassifier();
	~ArmorNumClassifier();

	/**
	 * @brief: load the SVM model used to recognize armorNum 载入SVM模型（用于识别装甲板数字）
	 * @param: the path of xml_file, the size of the training dataset ImgSize  待载入SVM模型的路径 模型的图片尺寸
	 */
	void loadSvmModel(const char* model_path, Size armorImgSize = Size(40, 40));

	/**
	 * @brief: load the current roiImage from ArmorDetector 载入roiImage（剪切出装甲板）
	 * @param: the path of xml_file  待载入SVM模型的路径
	 */
	void loadImg(Mat& srcImg);

	/**
	 * @brief: use warpPerspective to get armorImg  利用透视变换获得装甲板图片
	 * @param: the path of xml_file  待载入SVM模型的路径
	 */
	void getArmorImg(ArmorBox& armor);

	/**
	 * @brief: use SVM to recognize the number of each Armor 利用SVM实现装甲板数字识别
	 */
	void setArmorNum(ArmorBox& armor);

};

//#############################################  add-end

/************************************************
 * @brief 装甲识别部分
 ************************************************/
class ArmorDetector{

public:	
	EnemyColor enemyColor;			//敌方装甲板灯条的颜色
	
	int rgb_threshold;				//图像通道相减的阈值
	int tmp_threshold;				//模板匹配数字的阈值
	
	Mat srcImg;						//原图像
	Mat binImg;						//二值化后的图像
	Mat tmpImg;						//模板匹配的二值化后的图像

	vector<LightBar> lights;		//找到的所有灯条
	vector<ArmorBox> armors;		//找到的所有装甲板

	ArmorBox targetArmor;			//目标装甲板

	DetectorState state;			//识别状态

	//#############################################  add-begin

	int targetNum;					//操作手设置的目标装甲板
	ArmorNumClassifier classifier;	//获取装甲板图像及识别装甲板数字的类

	//#############################################  add-end


public:
	/**
	 * @brief 有参构造函数
	 * 
	 * @param enemyColor 敌方装甲板颜色
	 * @param camera_src 相机读取的源图像
	 * @param rgb_thre	 通道相减阈值
	 * @param svm_thre	 svm阈值
	 */
	ArmorDetector( EnemyColor enemyColor, int rgb_thre=70, int svm_thre=60 );

	/**
	 * @brief 重置装甲板的各项信息，清空上一帧信息
	 */
	void resetDetector();

	/**
     *@brief: return the Detector status 识别程序是否识别到装甲版
     *@return: FOUND(1) NOT_FOUND(0)
     */
    bool isFoundArmor();

	/////////////////////////////////////图像预处理部分: ImgProcesss.cpp/////////////////////////////////
	/**
	 *@brief 图像预处理部分：图像通道相减
	 */
	void rgbImgProcess();

	void showProcessedRgbImg();

	/**
	 * @brief 图像预处理部分：svm提取数字特征，仿射变换
	 */
	void tmpImgProcess();

	void showProcessedTmpImg();
	
	/**
	 * @brief 图像预处理部分：rgb+tmp
	 */
	void imgProcess( Mat & camera_src );

	//////////////////////////////////灯条筛选与装甲板识别部分: ArmorDetector.cpp////////////////////////////
	/**
	 * @brief 遍历图像中的所有轮廓，筛选可能的灯条轮廓
	 */
	void findLights();

	/**
	 * @brief 根据得到的灯条进行装甲板的拟合
	 */
	void matchArmors(int shriTimes);
	//#############################################  add-begin
	/**
	* @brief 负载svm模型
	* @param  the model file path of svm 路径
	*/
	void loadSVM(const char* model_path, Size armorImgSize = Size(40, 40));

	///**
	//*@brief: for client, set the target armor number #######操作手用，设置目标装甲板数字
	// */
	//void setTargetNum(const int& targetNum);

	//############################################   add-end

	/**
	 * @brief 选择目标装甲板
	 */
	void setTargetArmor();

	/**
	 * @brief 进行装甲识别与追踪：imgProcess->findLights->matchArmors
	 */
	void detecting( Mat & camera_src, int shriTimes );	
	
	/////////////////////////////////////Debug部分：显示识别到的灯条、装甲板等///////////////////////////////
	/**
 	 *@brief: show all the lights found in a copy of srcImg  在图像中显示找到的所有灯条
 	 */
	void showLights();

	/**
 	 *@brief: show all the armors matched in a copy of srcImg  在图像中显示找到的所有装甲板
 	 */
	void showArmors();

	/**
	 * @brief 显示一些处理结果
	 * 
	 * @param showProcedRbg 1-显示，0-不显示 
	 * @param showProcedTmp 
	 * @param showLights 
	 * @param showArmos 
	 */
	void showDebugInfo( int ShowProcedRbg = 1, int ShowProcedTmp = 1, int ShowLights = 1, int ShowArmos = 1 );

};



#endif
