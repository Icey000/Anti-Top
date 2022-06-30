#include <ArmorDetector/ArmorDetector.cpp>
#include <Position/Monocular.cpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include <iostream>
#include <fstream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <zero/cvConfig.h>
#include <ArmorDetector/AntiTop.hpp>
#include <ArmorDetector/Frame.hpp>
#include <serial.cpp>
#include <Guide.cpp>
#include <std_msgs/Float64MultiArray.h>
#include <sys/time.h>

Mat srcImg;
Monocular posi;
int flag = 0,KEY = 1,flagggggggggggg=0;
double dt=0.2,l=1,u=1,v=15;
std::pair<double,double> push;
bool flagg=0;
ArmorBox last;





void imageCallback(const sensor_msgs::ImageConstPtr& msg){
	cv_bridge::CvImagePtr cv_ptr;
	try{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	flag+=1;
	srcImg=cv_ptr->image;
}

void inputTips(){
    cout<<"____________________________________________________"<<endl;
    cout<<"----------Please Input Enemy Armor's Color----------"<<endl;
    cout<<"-------------------1 For Red----------------------"<<endl;
    cout<<"-------------------2 For Blue---------------------"<<endl;
    cout<<"____________________________________________________"<<endl;
}

void endTips(){
    cout<<"____________________________________________________"<<endl;
    cout<<"----------------Image Is Terminated-----------------"<<endl;
    cout<<"____________________________________________________"<<endl;
}

void dy_callback(zero::cvConfig &config, uint32_t level) 
{
	KEY=config.KEY;kk=config.kk;
	v=config.v;dt=config.dt;
	l=config.l;u=config.u;
	pid_dt=config.pid_dt;
	pid_max=config.pid_max;
	pid_min=config.pid_min;
	pid_Kp=config.pid_Kp;
	pid_Ki=config.pid_Ki;
	pid_Kd=config.pid_Kd;
} 

void Guide()
{
	double x,y,z,x0,v0,z0;
	x0=posi.x_pos/1000;v0=posi.z_pos/1000;z0=-posi.y_pos/1000;// 米转化为毫米
	x=x0;y=v0-0.1350;z=z0+0.1025;// 转换成枪口坐标系
	if(x0==8.888&&v0==8.888&&z0==-8.888){x=x_temp;y=y_temp;z=z_temp;flagg=1;flagggggggggggg=0;}
	else flagg=0;// 接收到8.8880的话，就使用上一次传过来的物体位置信息
	GuideCallback(SerialMain());
	push=Guidemain(flagg,x,y,z,dt,l,u,v);
}


int main( int argc,char **argv ){
   // VideoCapture cap("/home/rm/standard_rm/src/cv/Framedrop.avi");

    ros::init(argc,argv,"Detector");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    image_transport::ImageTransport it_(n);
    image_transport::Subscriber image_sub = it_.subscribe("camera/image_raw", 10, imageCallback);

    // 动态调参
    dynamic_reconfigure::Server<zero::cvConfig> server;
    dynamic_reconfigure::Server<zero::cvConfig>::CallbackType f;
    f = boost::bind(&dy_callback, _1, _2);
    server.setCallback(f);

    //inputTips();
    EnemyColor enemyColor;
    //cin >> color;
    //n.getParam("color",color);
    int color = 1;
    if( color == 1 ) enemyColor = RED;
    else if( color == 2 ) enemyColor = BLUE;

    ArmorDetector armorDetector( enemyColor );
   ArmorBox tmpArmor;
    //ArmorBox last,target;

    int shriTimes = 3;
    //OpenSerial();

	   
    while( ros::ok() ){

	//cap.read(srcImg);	
	struct timeval tv_begin, tv_end;
	double fps = 0;
        if( flag > 1 ){
            
            if( srcImg.empty() ){
                endTips();
                break;
            }
	gettimeofday(&tv_begin, NULL);  
            resize(srcImg, srcImg, Size(srcImg.size().width/shriTimes, srcImg.size().height/shriTimes));
            armorDetector.detecting( srcImg, shriTimes );
	    ///////////////////////反陀螺部分/////////////////////////
            if (armorDetector.state == ARMOR_FOUND)
            {
                HistoryTargets.emplace_front(armorDetector.targetArmor);
            }
            if(AntiTop(armorDetector))
            {
                cout<<"Top!!!"<<endl;//反陀螺击打
            }
            if(HistoryTargets.size() > 5)
            {
                HistoryTargets.pop_back();
            }
            ////////////////////////////////////////////////////////////
            if (armorDetector.state == ARMOR_FOUND)
           {
		for (int i = 0; i < armorDetector.armors.size(); i++)
               {
                   HistoryTarget.emplace_front(armorDetector.targetArmor);
               }
               
           } 

            
            ///////////////////////定位部分/////////////////////////
            Framedrop(armorDetector);
            if( armorDetector.state == ARMOR_FOUND )
             { 
               if(drop == 1)
               {  
                        last = HistoryTarget[1];
			
			posi.setArmorSize(last.type);
		        last.reset2DPoints( shriTimes );
		        posi.setTarget2D( last.armorVertices, last.center, last.type );
		        posi.getArmorCoordinate();
		        posi.showDebugInfo(1, 1);

		  cout << "temporary Not Detected Armors!" << endl;
               }
               else
               {        
                        
		        tmpArmor = armorDetector.targetArmor;
			
			    posi.setArmorSize(tmpArmor.type);
		        tmpArmor.reset2DPoints( shriTimes );
		        posi.setTarget2D( tmpArmor.armorVertices, tmpArmor.center, tmpArmor.type );
		        posi.getArmorCoordinate();
		        posi.showDebugInfo(1, 1);
               }
		        //tmpArmor.recover2DPoints( shriTimes );
			}
            else{
                        

            //             target = HistoryTarget[0];
			
			// posi.setArmorSize(target.type);
		    //     target.reset2DPoints( shriTimes );
		    //     posi.setTarget2D( target.armorVertices, target.center, target.type );
		    //     posi.getArmorCoordinate();
		    //     posi.showDebugInfo(1, 1);
                         posi.x_pos = 8888;
		         posi.z_pos = 8888;
		         posi.y_pos = 8888;
                         // tmpArmor = armorDetector.targetArmor;
			
			   // posi.setArmorSize(tmpArmor.type);
		        //tmpArmor.reset2DPoints( shriTimes );
		        //posi.setTarget2D( tmpArmor.armorVertices, tmpArmor.center, tmpArmor.type );
		        //posi.getArmorCoordinate();
		        //posi.showDebugInfo(1, 1);

		            	cout << "Not Detected Armors!" << endl;
			}
			
		    ////////////////////////////////////////////////////////
	/*	//recording data
		char a[10],b[10],c[10];
		sprintf(a,"%f",msg.x);
		sprintf(b,"%f",msg.y);
		sprintf(c,"%f",msg.z);
		out.write(a,5);
		out.write("\n",1);
		out.write(b,6);
		out.write("\n",1);
		out.write(c,5);
		out.write("\n\n",2);	*/
            //////////////////////计算帧率 && DEBUG//////////////////////////
        //    current_time = clock();
	gettimeofday(&tv_end, NULL);
           double time_cost = (double)(1000000 * (tv_end.tv_sec - tv_begin.tv_sec)  + (tv_end.tv_usec - tv_begin.tv_usec) )/ CLOCKS_PER_SEC * 1000;
            fps = 1000 / time_cost;
	cout<<"fps@@@@@@@@@@@@@"<<fps;
          //  putText( srcImg, to_string(fps), Point(100, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 255), 1, 8, false );
	    imshow("srcImg", srcImg);
            armorDetector.showDebugInfo(1, 0, 1, 1);
            waitKey(KEY);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		if(flagggggggggggg<1)
			for(int o=0;o<10;o++)	
				Guide();
		flagggggggggggg++;
		Guide();
		//if(flagggggggggggg>20)
			//PUSH(flagg,push.first,push.second);
        }

        ros::spinOnce();
    	loop_rate.sleep();              
    }

    destroyAllWindows();
    ser.close();
    return 0;

}

