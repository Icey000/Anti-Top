#include "ArmorDetector.h"
#include "ImageProcess.cpp"
#include "LightBar.cpp"
#include "ArmorBox.cpp"
#include <cv/yapi.h> 

int X_point,Y_point;
#define horiz_screen_size 1440
#define vert_screen_size 540


/**
 * @brief 针对游离灯条导致的错误装甲板进行检测和删除
 * 
 * @param armors 装甲板集合
 */
void eraseErrorRepeatArmor(vector<ArmorBox> & armors);

/**
 * @brief 有参构造函数
 * 
 * @param enemyColor 敌方装甲板颜色
 * @param camera_src 相机读取的源图像
 * @param rgb_thre	 通道相减阈值
 * @param tmp_thre	 模板匹配阈值
 */
ArmorDetector::ArmorDetector( EnemyColor enemyColor, int rgb_thre, int tmp_thre ){
    this->enemyColor = enemyColor;
    this->rgb_threshold = rgb_thre;
    this->tmp_threshold = tmp_thre;    
	this->state = ARMOR_NOT_FOUND;
}

/**
 * @brief 重置装甲板的各项信息，清空上一帧信息
 */
void ArmorDetector::resetDetector()
{
	state = LIGHTS_NOT_FOUND;
	lights.clear();
	armors.clear();
}

/**
 *@brief: return the Detector status 识别程序是否识别到装甲版
 *@return: FOUND(1) NOT_FOUND(0)
 */
bool ArmorDetector::isFoundArmor(){
	if( this->state == ARMOR_FOUND ){
		return 1;
	}else{
		return 0;
	}
}

/**
 * @brief 遍历图像中的所有轮廓，筛选可能的灯条轮廓
 */
void ArmorDetector::findLights(){
    vector<vector<Point>> lightContours;  //candidate contours of lights roiIng中的候选灯条轮廓
	Mat contourImg; //image for the useage of findContours avoiding the unexpected change of itself 给findContours用的图像，防止findContours改变roiImg
	binImg.copyTo(contourImg); //a copy of roiImg, contourImg
	findContours(contourImg, lightContours, 0, 2); //CV_RETR_EXTERNAL = 0, CV_CHAIN_APPROX_SIMPLE = 2       最耗时的操作，优化方向
	RotatedRect lightRect;  //RotatedRect for fitEllipse 拟合椭圆来的灯条旋转矩形
	LightBar light;  //template light 临时灯条
	for (const auto& lightContour : lightContours) {

			//cout<<" 面积筛选"<<contourArea(lightContour)<<endl; 
		if (lightContour.size() < 6) continue; //if contour's size is less than 6 , then it can not used to fitEllipse 轮廓点数小于6，不可拟合椭圆
		if (contourArea(lightContour) < armorParam.min_area){continue;} //minarea of lightContour to filter some small blobs 面积筛选滤去小发光点


		lightRect = fitEllipse(lightContour); //lightContour fits into a RotatedRect 拟合椭圆
		light = LightBar(lightRect);//construct to a lightBar 构造为灯条

		if (abs(light.angle) > armorParam.max_angle)continue; //angle filter 角度筛选，滤去一些竖直偏角偏大的
//	cout<<"竖直偏角"<<light.angle<<endl; 
		lights.emplace_back(light); //使用emplace_back比使用push_back更快
	}
	if (lights.size() < 2) {
		state = LIGHTS_NOT_FOUND; //if lights is less than 2, then set state not found lights 灯条少于两条则设置状态为没找到灯条
		return; //exit
	}

	// sort the lightBars from left to right 将灯条从左到右排序
	sort( lights.begin(), lights.end(), [](LightBar & a1, LightBar & a2) { return a1.center.x < a2.center.x; } );
	state = LIGHTS_FOUND;
	return;
}

/**
* @brief: match lights into armors 将识别到的灯条拟合为装甲板
*/
/*
void ArmorDetector::matchArmors(int shriTimes){
	for (int i = 0; i < lights.size() - 1; i++)
	{
		for (int j = i + 1; j < lights.size(); j++) //just ensure every two lights be matched once 从左至右，每个灯条与其他灯条一次匹配判断
		{
				ArmorBox armor = ArmorBox(lights[i], lights[j], shriTimes); //construct an armor using the matchable lights 利用左右灯条构建装甲板		
				classifier.getArmorImg(armor);// set armor image 装甲板的二值图
				//	classifier.setArmorNum(armor);//set armor number 装甲板数字	
				if (armor.isSuitableArmor()) //when the armor we constructed just now is a suitable one,set extra information of armor 如果是合适的装甲板，则设置其他装甲板信息
				{
					armor.l_index = i; //set index of left light 左灯条的下标
					armor.r_index = j; //set index of right light 右灯条的下标	
								
					armor.calcuArmorArea(); //计算装甲板面积
					armors.emplace_back(armor); //push into armors 将匹配好的装甲板push入armors中
				}
		}

		eraseErrorRepeatArmor(armors);//delete the error armor caused by error light 删除游离灯条导致的错误装甲板
	}
	if (armors.empty()) {
		state = ARMOR_NOT_FOUND; //if armors is empty then set state ARMOR_NOT_FOUND 如果armors目前仍为空，则设置状态为ARMOR_NOT_FOUND
		return; //exit function
	} 
	else {
		state = ARMOR_FOUND; //else set state ARMOR_FOUND 如果非空（有装甲板）则设置状态ARMOR_FOUND
		return; //exit function
	}
}
*/

//xieomeng
void ArmorDetector::matchArmors(int shriTimes){
	for (int i = 0; i < lights.size() - 1; i++)
	{
		for (int j = i + 1; j < lights.size(); j++) //just ensure every two lights be matched once 从左至右，每个灯条与其他灯条一次匹配判断
		{
				ArmorBox armor = ArmorBox(lights[i], lights[j], shriTimes); //construct an armor using the matchable lights 利用左右灯条构建装甲板		
				
				if (armor.isSuitableArmor()) //when the armor we constructed just now is a suitable one,set extra information of armor 如果是合适的装甲板，则设置其他装甲板信息
				{
					//classifier.getArmorImg(armor);// set armor image 装甲板的二值图
					//classifier.setArmorNum(armor);//set armor number 装甲板数字
					//if(armor.armorNum==1){

					armor.l_index = i; //set index of left light 左灯条的下标
					armor.r_index = j; //set index of right light 右灯条的下标
										
					armor.calcuArmorArea(); //计算装甲板面积
					armors.emplace_back(armor); //push into armors 将匹配好的装甲板push入armors中
					//}
				}
		} 

		eraseErrorRepeatArmor(armors);//delete the error armor caused by error light 删除游离灯条导致的错误装甲板
	}
	if (armors.empty()) {
		state = ARMOR_NOT_FOUND; //if armors is empty then set state ARMOR_NOT_FOUND 如果armors目前仍为空，则设置状态为ARMOR_NOT_FOUND
		return; //exit function
	} 
	else {
		state = ARMOR_FOUND; //else set state ARMOR_FOUND 如果非空（有装甲板）则设置状态ARMOR_FOUND
		return; //exit function
	}
}

/*
//chenxin
void ArmorDetector::matchArmors(int shriTimes){
	for (int i = 0; i < lights.size() - 1; i++)
	{
		for (int j = i + 1; j < lights.size(); j++) //just ensure every two lights be matched once 从左至右，每个灯条与其他灯条一次匹配判断
		{
				ArmorBox armor = ArmorBox(lights[i], lights[j], shriTimes); //construct an armor using the matchable lights 利用左右灯条构建装甲板		
				
				if (armor.isSuitableArmor()) //when the armor we constructed just now is a suitable one,set extra information of armor 如果是合适的装甲板，则设置其他装甲板信息
				{
					armor.l_index = i; //set index of left light 左灯条的下标
					armor.r_index = j; //set index of right light 右灯条的下标	
					classifier.getArmorImg(armor);// set armor image 装甲板的二值图
					classifier.setArmorNum(armor);//set armor number 装甲板数字				
					armor.calcuArmorArea(); //计算装甲板面积
					armors.emplace_back(armor); //push into armors 将匹配好的装甲板push入armors中
				}
		}

		eraseErrorRepeatArmor(armors);//delete the error armor caused by error light 删除游离灯条导致的错误装甲板
	}
	if (armors.empty()) {
		state = ARMOR_NOT_FOUND; //if armors is empty then set state ARMOR_NOT_FOUND 如果armors目前仍为空，则设置状态为ARMOR_NOT_FOUND
		return; //exit function
	} 
	else {
		for(auto& armor:armors){
			if(armor.armorNum==1){
			state=ARMOR_FOUND;
			return;
			}
		}
		state = ARMOR_FOUND; //else set state ARMOR_FOUND 如果非空（有装甲板）则设置状态ARMOR_FOUND
		return; //exit function
	}
}
*/

/**
 * @brief 选择目标装甲板
 */
void ArmorDetector::setTargetArmor(){
	int maxArea = 0, maxNo = 0;
	for( int i = 0; i<this->armors.size(); ++i ){
		if (armors[i].armorArea > maxArea ){
			maxArea = armors[i].armorArea;
			maxNo = i;
		}
	}
	this->targetArmor = armors[maxNo];
}

/**
 * @brief 进行装甲识别与追踪：imgProcess->findLights->matchArmors
 * 
 * @param camera_src 源图像
 */
void ArmorDetector::detecting( Mat & camera_src, int shriTimes ){
	//firstly, load and set srcImg  首先，载入并处理图像
	this->imgProcess(camera_src); //globally srcImg and preprocess it into srcImg_binary 载入Detector的全局源图像 并对源图像预处理成

	//secondly, reset detector before we findLights or matchArmors(clear lights and armors we found in the last frame and reset the state as LIGHTS_NOT_FOUND) 
	//随后，重设detector的内容，清空在上一帧中找到的灯条和装甲板，同时检测器状态重置为LIGHTS_NOT_FOUND（最低状态）
	resetDetector();

	//thirdly, find all the lights in the current frame (srcImg)
	//第三步，在当前图像中找出所有的灯条
	findLights();

	//forthly, if the state is LIGHTS_FOUND (detector found more than two lights) , we match each two lights into an armor
	//第四步，如果状态为LIGHTS_FOUND（找到多于两个灯条），则
	if (state == LIGHTS_FOUND)
	{
		//match each two lights into an armor and if the armor is a suitable one, emplace back it into armors
		//将每两个灯条匹配为一个装甲板，如果匹配出来的装甲板是合适的，则压入armors中
		matchArmors(shriTimes);

		//if the state is ARMOR_FOUND(detector has matched suitable armor), set target armor and last armor
		//如果找到了灯条，则设置好目标装甲板和上一个装甲板
		if (state == ARMOR_FOUND) {
			setTargetArmor();
		}
	}

}

/**
 *@brief: detect and delete error armor which is caused by the single lightBar 针对游离灯条导致的错误装甲板进行检测和删除
 */
void eraseErrorRepeatArmor(vector<ArmorBox> & armors)
{
	int length = armors.size();
	vector<ArmorBox>::iterator it = armors.begin();
	for (size_t i = 0; i < length; i++)
		for (size_t j = i + 1; j < length; j++)
		{
			if (armors[i].l_index == armors[j].l_index ||
				armors[i].l_index == armors[j].r_index ||
				armors[i].r_index == armors[j].l_index ||
				armors[i].r_index == armors[j].r_index)
			{
				armors[i].getDeviationAngle() > armors[j].getDeviationAngle() ? armors.erase(it + i) : armors.erase(it + j);
			}
		}
}

/**
 *@brief: show all the lights found in a copy of srcImg  在图像中显示找到的所有灯条
 */
void ArmorDetector::showLights()
{
	Mat lightDisplay;//image for the use of dialaying the lights 显示灯条用的图像
	this->srcImg.copyTo(lightDisplay);//get a copy of srcImg 获取源图像的拷贝
	//if detector finds lights 如果找到了灯条
	if (!this->lights.empty())
	{
		putText(lightDisplay, "LIGHTS FOUND!", Point(100, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 255), 1, 8, false); //title LIGHT_FOUND 大标题 “找到了灯条”
		for (auto light : lights)
		{
			Point2f lightVertices[4];
			light.lightRect.points(lightVertices);
			//draw all the lights' contours 画出所有灯条的轮廓
			for (size_t i = 0; i < 4; i++)
			{
				line(lightDisplay, lightVertices[i], lightVertices[(i + 1) % 4], Scalar(255, 0, 255), 1, 8, 0);
			}

			//draw the lights's center point 画出灯条中心
			circle(lightDisplay, light.center, 2, Scalar(0, 255, 0), 2, 8, 0);

			//show the lights' center point x,y value 显示灯条的中心坐标点
			putText(lightDisplay, to_string(int(light.center.x)), light.center, FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0), 1, 8, false);
			putText(lightDisplay, to_string(int(light.center.y)), light.center + Point2f(0, 15), FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0), 1, 8, false);
		}
	}
	//if detector does not find lights 如果没找到灯条
	else
	{
		putText(lightDisplay, "LIGHTS NOT FOUND!", Point(100, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 1, 8, false);//title LIGHT_NOT_FOUND 大标题 “没找到灯条”
	}
	//show the result image 显示结果图
	imshow("Lights Monitor", lightDisplay);
}

/**
 *@brief: show all the armors matched in a copy of srcImg  在图像中显示找到的所有装甲板
 */
void ArmorDetector::showArmors()
{
	Mat armorDisplay; //Image for the use of displaying armors 展示装甲板的图像
	this->srcImg.copyTo(armorDisplay); //get a copy of srcImg 源图像的拷贝
	circle(armorDisplay, Point2f(armorDisplay.size().width/2, armorDisplay.size().height/2), 4, Scalar(255, 255, 255), 2 );  
	// if armors is not a empty vector (ARMOR_FOUND) 如果找到了装甲板
	if (!this->armors.empty())
	{
		putText(armorDisplay, "ARMOR FOUND!", Point(100, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 0), 1, 8, false); //title FOUND 大标题 “找到了装甲板”
		//draw all the armors' vertices and center 画出所有装甲板的顶点边和中心
		for (auto armor : armors)
		{
			//draw the center 画中心
			circle(armorDisplay, armor.center, 2, Scalar(0, 255, 0), 2);
			for (size_t i = 0; i < 4; i++)
			{
				line(armorDisplay, armor.armorVertices[i], armor.armorVertices[(i + 1) % 4], Scalar(255, 255, 0), 2, 8, 0);
			}
			//display its center point x,y value 显示中点坐标
			putText(armorDisplay, to_string(int(armor.center.x)), armor.center, FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 255), 1, 8, false);
			putText(armorDisplay, to_string(int(armor.center.y)), armor.center + Point2f(0, 15), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 255), 1, 8, false);
            putText(armorDisplay, to_string(int(armor.armorNum)), armor.center + Point2f(15, 30), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255), 1, 8, false);
		}
		//connect all the vertices to be the armor contour 画出装甲板轮廓
		for (size_t i = 0; i < 4; i++)
		{
			line(armorDisplay, targetArmor.armorVertices[i], targetArmor.armorVertices[(i + 1) % 4], Scalar(255, 255, 255), 2, 8, 0);
		}
		//draw shot point for Guide
	}
	//if armors is a empty vector (ARMOR_NOT FOUND) 如果没找到装甲板
	else
	{
		putText(armorDisplay, "ARMOR NOT FOUND!", Point(100, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 1, 8, false);//title NOT FOUND 大标题 “没找到装甲板”
	}
	//show the result armors image 显示结果图
	imshow("Armor Monitor", armorDisplay);
}

/**
 * @brief 显示一些处理结果
 * 
 * @param showProcedRbg 1-显示，0-不显示 
 * @param showProcedTmp 
 * @param showLights 
 * @param showArmos 
 */
void ArmorDetector::showDebugInfo( int ShowProcedRbg, int ShowProcedTmp, int ShowLights, int ShowArmors ){
	if( ShowProcedRbg ) this->showProcessedRgbImg();
	//if( ShowProcedTmp ) this->showProcessedTmpImg();
	if( ShowLights ) this->showLights();
	if( ShowArmors ) this->showArmors();
}



