#include "ArmorDetector.h"

/**
 *@brief: calculate the cross point of four points in order bl(below left),tl(top left),tr(top right),br(below right)
 */
const Point2f crossPointof(const Point2f& bl,const Point2f& tl,const Point2f& tr, const Point2f& br){
	float a1 = tr.y - bl.y;
	float b1 = tr.x - bl.x;
	float c1 = bl.x*tr.y - tr.x*bl.y;

	float a2 = br.y - tl.y;
	float b2 = br.x - tl.x;
	float c2 = tl.x*br.y - br.x*tl.y;

	float d = a1 * b2 - a2 * b1;

	if (d == 0.0){
		return Point2f(FLT_MAX, FLT_MAX);
	}
	else{
		return cv::Point2f((b2*c1 - b1 * c2) / d, (c1*a2 - c2 * a1) / d);
	}
}

/**
 *@brief: using the lightRect of two lightBar to construct the armorVertices
 */
void setArmorVertices(const LightBar & l_light, const LightBar & r_light, ArmorBox & armor) {
	//handle two lights
	cv::Size exLSize((l_light.lightRect.size.width), (l_light.lightRect.size.height ));
	cv::Size exRSize((r_light.lightRect.size.width), (r_light.lightRect.size.height ));
	cv::RotatedRect exLLight(l_light.center, exLSize, armor.armorAngle);
	cv::RotatedRect exRLight(r_light.center, exRSize, armor.armorAngle);

	cv::Point2f pts_l[4];
	exLLight.points(pts_l);
	cv::Point2f upper_l =(pts_l[1]+pts_l[2])/2;
	cv::Point2f lower_l =(pts_l[0]+pts_l[3])/2;

	cv::Point2f pts_r[4];
	exRLight.points(pts_r);
	cv::Point2f upper_r =(pts_r[2]+pts_r[1])/2;
	cv::Point2f lower_r =(pts_r[0]+pts_r[3] )/2;
	
        //装甲板的四个顶点
	armor.armorVertices[0] = upper_l;
	armor.armorVertices[1] = upper_r;
	armor.armorVertices[2] = lower_r;
	armor.armorVertices[3] = lower_l;
}

ArmorBox::ArmorBox() {
	l_index = -1;
	r_index = -1;
	l_light = LightBar();
	r_light = LightBar();
	armorNum = 0;
	armorVertices.resize(4);
	type = SMALL_ARMOR;
	center = Point2f();
	armorRect = Rect();
	armorImg = Mat();
}

ArmorBox::ArmorBox(const LightBar & l_light, const LightBar & r_light, int shriTimes) {
	this->l_light = l_light;
	this->r_light = r_light;
	
	armorNum = 0;
	armorAngle = (l_light.angle + r_light.angle) / 2;
	
	//set armorVertices tl->tr->br->bl      左上 右上 右下 左下
	armorVertices.resize(4);
	setArmorVertices(l_light, r_light, *this);    // '*this' means the reference of this ArmorBox

	//set armor center
	center = crossPointof(armorVertices[0], armorVertices[1], armorVertices[2], armorVertices[3]);

	//set armorRect using boundingRect for convenience
	armorRect = boundingRect(armorVertices);


       float lightX = (armorVertices[1].x -armorVertices[0].x + armorVertices[2].x - armorVertices[3].x) /2;
       float lightY = (armorVertices[3].y -armorVertices[0].y + armorVertices[2].y - armorVertices[1].y) /2;

	//set armor type according to its x ration
 if ( (lightX / lightY) > 3.3 ){
     	   type = BIG_ARMOR;
 	   }
  	  else if ( (lightX / lightY) > 2.6){
        if( lightX > 130){
            type = BIG_ARMOR;
        }
        else {
            type = SMALL_ARMOR;
            }
  	  }
   	 else type = SMALL_ARMOR;
}


ArmorBox::~ArmorBox() {}

// angle difference: the angle difference of left and right lights 装甲板左右灯条角度差
float ArmorBox::getAngleDiff() const {
	float angle_diff = abs(l_light.angle - r_light.angle); //get the abs of angle_diff 灯条的角度差
	//cout<<"装甲板左右灯条角度差:"<<angle_diff<<endl;
	return angle_diff;
}

// deviation angle : the horizon angle of the line of centers of lights 灯条错位度角(两灯条中心连线与水平线夹角) 
float ArmorBox::getDeviationAngle() const {
	float delta_x = r_light.center.x - l_light.center.x; //Δx
	float delta_y = r_light.center.y - l_light.center.y; //Δy
	float deviationAngle = abs(atan(delta_y / delta_x)) * 180 / CV_PI; //tanθ=Δy/Δx
	//cout<<" 灯条错位度角:"<<deviationAngle<<endl;
	return deviationAngle;
}

// dislocation judge X: r-l light center distance ration on the X-axis 灯条位置差距 两灯条中心x方向差距比值
float ArmorBox::getDislocationX() const {
	float meanLen = (l_light.length + r_light.length) / 2;
	float xDiff = abs(l_light.center.x - r_light.center.x); //x distance ration y轴方向上的距离比值（y轴距离与灯条平均值的比）
	float xDiff_ratio = xDiff / meanLen;
	//cout<<"两灯条中心x方向差距比值:"<<xDiff_ratio<<endl;
	return xDiff_ratio;
}

// dislocation judge Y:  r-l light center distance ration on the Y-axis 灯条位置差距 两灯条中心Y方向差距比值
float ArmorBox::getDislocationY() const {
	float meanLen = (l_light.length + r_light.length) / 2;
	float yDiff = abs(l_light.center.y - r_light.center.y);  //y distance ration x轴方向上的距离比值（x轴距离与灯条平均值的比）
	float yDiff_ratio = yDiff / meanLen;
	//cout<<"两灯条中心y方向差距比值:"<<yDiff_ratio<<endl;
	return yDiff_ratio;
}

// length difference ration: the length difference ration r-l lights 左右灯条长度差比值
float ArmorBox::getLengthRation() const {
	float length_diff = abs(l_light.length - r_light.length);
	float lengthDiffRation = length_diff / MAX(l_light.length, r_light.length);
	//cout<<" 左右灯条长度差比值:"<<lengthDiffRation<<endl;
	return lengthDiffRation;
}

// judge whether this armor is suitable or not  判断本装甲板是否是合适的装甲板
bool ArmorBox::isSuitableArmor() const
{
	return
		this->getAngleDiff()	  < armorParam.max_angle_diff	    &&		// angle difference judge the angleDiff should be less than max_angle_diff 灯条角度差判断，需小于允许的最大角差
		this->getDeviationAngle() < armorParam.max_deviation_angle  &&		// deviation angle judge: the horizon angle of the line of centers of lights 灯条错位度角(两灯条中心连线与水平线夹角)判断 
		this->getDislocationX()	  < armorParam.max_x_diff_ratio     &&		// dislocation judge: the x and y can not be too far 灯条位置差距 两灯条中心x、y方向差距不可偏大（用比值作为衡量依据）
		this->getDislocationY()	  < armorParam.max_y_diff_ratio     &&		// dislocation judge: the x and y can not be too far 灯条位置差距 两灯条中心x、y方向差距不可偏大（用比值作为衡量依据）
		this->getLengthRation()   < armorParam.max_lengthDiff_ratio;
}

void ArmorBox::calcuArmorArea(){
	this->armorArea = this->armorRect.area();
}

void ArmorBox::reset2DPoints( int shriTimes ){
	this->center.x *= shriTimes;
	this->center.y *= shriTimes;
	for(int i = 0; i<4; i++){
		this->armorVertices[i].x *= shriTimes;
		this->armorVertices[i].y *= shriTimes;
	}
}

void ArmorBox::recover2DPoints( int shriTimes ){
	this->center.x /= shriTimes;
	this->center.y /= shriTimes;
	for(int i = 0; i<4; i++){
		this->armorVertices[i].x /= shriTimes;
		this->armorVertices[i].y /= shriTimes;
	}

}
