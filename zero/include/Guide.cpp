// 1.找数学模型（弹道补偿部分的可以套用，主要是目标预测）；
// 2.以代码形式实现需求；
// 3.优化代码，运行时间尽可能短；
// 4.调整卡尔曼滤波参数和其它参数；

// 最重要的：弄清楚坐标系

// 目前缺点：
// 1.缺少排除奇异值（目标变换时发生的和其他原因产生的，目前还不清楚）的影响方案；
// 2.参数（初速度、阻力系数、物体质量等等）还没统一，实际目标预测会有些偏差。

#include "std_msgs/String.h"
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

double after_x,after_y,after_z,x_temp,y_temp,z_temp,kk=0.05;
float wy,wp,_v,_theta;
int cycle=0;

MatrixXd A(6,6);MatrixXd H(3,6);MatrixXd P(6,6);MatrixXd Q(6,6);
MatrixXd R(3,3);VectorXd X(6);VectorXd X_hat(6);MatrixXd z_meas(3,1);
MatrixXd K(6,6);
char a[10];
	
// 动态调参，可以实时改变下面这四个参数的值（k阻力系数，dt时间间隔，l和u协方差矩阵的参数，一般初始值设置为1，随着滤波器迭代会之间变化）
// 软 rqt_reconfigure
// 接受从下位机传上来的云台角度信息
void GuideCallback(double angle[])  
{  
	//yaw_now=angle[0];pitch_now=angle[1];
	wy=angle[2];wp=angle[3];
	_v=angle[4];_theta=angle[5];
	//cout<<"I heard current_pitch:"<<pitch_now<<" ,current_yaw:"<<yaw_now<<endl;
	//cout<<"I heard wy:"<<wy<<" ,wp:"<<wp<<endl;
	//cout<<"I heard _v:"<<_v<<" ,_theta:"<<_theta<<endl;
}


// 弹道预测模型，主要是关于 pitch（俯仰角） 的预测
/**
*	@brief BulletModel 使用牛顿迭代法求解飞行时间T，最后得出物体理想情况下的垂直下落距离
*	@param v_target 在类平抛模型中，对物体水平速度的预测
*	@param y1 弹道模型得出的物体理想情况距原点的垂直高度（最后返回的值）
**/
double BulletModel(double x, double v, double angle, double &t) 
{	//x:m,v:m/s,m:kg,angle:rad
	double vx,vy,y1,g,t_k,log_result,f,f1,v_target;
	v_target = sqrt(pow(X[1], 2)+pow(X[3], 2));
	//v_target = 0;
	g = 9.8;
	vx = v * cos(angle);
	vy = v * sin(angle);

	t_k = 0;
	log_result = log(kk*vx*t_k+1);
	f = (1/kk)*log_result-x-v_target*t_k;
	f1 = vx/(kk*vx*t_k+1)-v_target;
	t = t_k - f/f1;

	while(t - t_k > 0.01)
	{
		t_k = t;
		log_result = log(kk*vx*t_k+1);
		f = (1/kk)*log_result-x-v_target*t_k;
		f1 = vx/(kk*vx*t_k+1)-v_target;
		t = t_k - f/f1;
	}

	// t = (exp(k*x)-1) / (k*vx);
	y1 = vy*t - g*t*t/2;
	return y1;
}
/**
*
*	@brief initialization 初始化卡尔曼滤波器的参数，暂时没有使用 tf 变换
*
**/

void initialization(double x,double y,double z,double dt,double l,double u,double v){
	A << 1, dt, 0, 0,  0, 0,
	     0, 1,  0, 0,  0, 0,
	     0, 0,  1, dt, 0, 0, 
	     0, 0,  0, 1,  0, 0,
	     0, 0,  0, 0,  1, dt,
	     0, 0,  0, 0,  0, 1;
	H << 1, 0, 0, 0, 0, 0,
	     0, 0, 1, 0, 0, 0,
	     0, 0, 0, 0, 1, 0;
	P << 1, 1, 0, 0, 0, 0,
	     1, 1, 0, 0, 0, 0,
	     0, 0, 1, 1, 0, 0,
	     0, 0, 1, 1, 0, 0,
	     0, 0, 0, 0, 1, 1,
	     0, 0, 0, 0, 1, 1;
	Q << 0, 0, 0, 0, 0, 0,
	     0, l, 0, 0, 0, 0,
	     0, 0, 0, 0, 0, 0,
	     0, 0, 0, l, 0, 0,
	     0, 0, 0, 0, 0, 0,
	     0, 0, 0, 0, 0, l;
	R << u, 0, 0,
	     0, u, 0,
	     0, 0, u;
	X << 0, 0, 0, 0, 0, 0; 
	X_hat<< x, 0, y, 0, z, 0;
}
/**
*
*	@brief average,standardDev计算平均值和标准差
*
**/
double average(const vector<double> &arr)
{
    double sum=0;
    for (unsigned i = 1; i < arr.size(); i++)
        sum += arr[i];
    return  sum / (arr.size()-1);//平均值
}

double standardDev(const vector<double> &arr,double average)
{
    double sum = 0,variance=0;
    for (unsigned i = 1; i < arr.size(); i++)
        sum += pow(arr[i] - average, 2);
    variance = sum/(arr.size()-1); // 方差
    return sqrt(variance);     //标准差
}

void Chi_squared_Test(vector<double> &arr,double &in,double vector_size)
{
    double ave,SD,range_up,range_down;
    arr.push_back(in);
    ave=average(arr);
    SD=standardDev(arr,ave);
    //.253-60//.524-70%//.842-80%
    range_up=ave+0.842*SD/sqrt(arr.size()-1);
    range_down=ave-0.842*SD/sqrt(arr.size()-1);
    
    if((in>range_up)||(in<range_down))
    {
    	in=arr[0];
    }	
    arr[0]=in;

    if((arr.size()-1)>=vector_size)
	arr.erase(arr.begin()+1,arr.begin()+2);
}

void CorrecteVel( float _theta, float wy, float wp, float _v ){
	float R1, R2, vx1, vy1, vx2, vz1;
	R1 = sqrt( pow( after_x, 2 ) + pow( after_y, 2 ) );
	R2 = sqrt( pow( after_y, 2 ) + pow( after_z, 2 ) );
	vx1 = wy * R1;
	vz1 = wp * R2;
	vx2 = _v * cos( _theta );
	vy1 = _v * sin( _theta );
	if( (X[1]-vx1) >= 0 && X[1] >= 0 || (X[1]-vx1) <= 0 && X[1] <=0 )
		X[1] = X[1] - vx1;
//	if( (X[3]-vy1) >= 0 && X[3] >= 0 || (X[3]-vy1) <= 0 && X[3] <= 0 )
//		X[3] = X[3] - vy1;
	if( (X[5]-vz1) >= 0 && X[5] >= 0 || (X[5]-vz1) <= 0 && X[5] <= 0 )
		X[5] = X[5] - vz1;
}

std::pair<double,double> Guidemain(bool flagg,double x,double y,double z,double dt,double l,double u,double v)
{	double x_2,y_2,yaw,pitch,y_actual,dy,y_temp2;
	vector<double> angle_pitch(1,0),angle_yaw(1,0); 
	std::pair<double,double> pub;
	//cnt++;
	//ros::Time begin = ros::Time::now();
	if(flagg==1)	
		return std::make_pair(0,0);
	else{
		if (cycle==0)
			initialization(x,y,z,dt,l,u,v);
		cycle++;
		z_meas<<x,y,z;

		X_hat(1)=(x-x_temp)/dt;X_hat(3)=(y-y_temp)/dt;X_hat(5)=(z-z_temp)/dt;
		X_hat = A * X_hat;
		Eigen::MatrixXd A_T = A.transpose();
		P = A * P * A_T + Q;
		Eigen::MatrixXd temp1,temp2,Ht;
		Ht = H.transpose();
		temp1 =  H * P * Ht + R;
		temp2 = temp1.inverse();
		K = P * Ht * temp2;
		Eigen::VectorXd Z = H * X_hat;
		X = X_hat + K * (z_meas - Z);
		Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6,6);
		P = (I - K * H) * P;		
    		after_x = X(0);after_y = X(2);after_z = X(4);
		// 到此为止为一次
		// _theta 为小车速度方向的夹角，wy云台yaw轴角速度，wp为云台pitch轴角速度，_v为小车移动速度，都是float
		CorrecteVel( _theta, wy, wp, _v );
	
		x_temp=after_x;y_temp=after_y;z_temp=after_z;
		/**
		*	@param y_2 抛物模型中物体物体在垂直方向上的距离
		*	@param x_2 水平方向上的额距离
		*	@param y_actual 弹道模型计算出来的弹丸落点高度（在给定 pitch 角度下的）
		**/
		y_2 = after_z;
		x_2 = sqrt(after_x*after_x+after_y*after_y);
		pitch = atan2(y_2,x_2);
		double t, vOfTarget = sqrt(pow(X[1], 2)+pow(X[3], 2));
		// y_2 = z;
		// x_2 = sqrt(x*x+y*y);
		//double t, vOfTarge
		t = 0;
		y_temp2 = y_2;
		// 在测试中循环在2~3次就收敛了
		for (int i = 0; i < 20; i++)
		{
			y_actual = BulletModel(x_2,v,pitch,t);
			dy = y_temp2 + X[5]*t - y_actual;
			// dy = y_temp2 - y_actual;
			y_temp2 = y_2 + dy;
			pitch = atan2(y_temp2,x_2 + vOfTarget*t);
			// pitch = atan2(y_temp2,x_2);
			if (fabs(dy)<0.01)
			{
				break;
			}
		}
		code_t2 = t;
		yaw = -atan2(after_x + X[1]*t,after_y + X[3]*t);
		// yaw = -atan2(after_x,after_y);
		//yaw = -atan2(x,y);
		pub = std::make_pair(pitch/3.14*180,yaw/3.14*180);
		//卡方检验
		Chi_squared_Test(angle_pitch,pub.first,5);
		Chi_squared_Test(angle_yaw,pub.second,5);
		//angle_pub.publish(angle);


		setlocale(LC_ALL, "");
		return pub;
	}
		//ROS_INFO("目标所在位置  x:  %.2f  y:  %.2f  z:  %.2f", after_x, after_y, after_z);
		// ROS_INFO("当前航向角: %.4f", yaw_now);
		// ROS_INFO("当前俯仰角: %.4f", pitch_now);
		//ROS_INFO("计算所得的航向角: %.4f", yaw/3.14*180);
		//ROS_INFO("计算所得的俯仰角: %.4f", pitch/3.14*180);
		//ROS_INFO("本次弹丸飞行时长：%.6f", t);
		//ros::Time end = ros::Time::now();
		//timeSum += (end.sec - begin.sec);
		//ROS_INFO("每次的平均时间: %.4f", timeSum/cnt);

}
