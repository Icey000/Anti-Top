#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <serial/serial.h>
#include <iostream>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
using namespace std;


#define receivelength 14	//*******按照收数据(short int)的数目进行修改,具体为 (消息个数)*2+2
#define sendlength 10		//*******按照发数据(short int)的数目进行修改,具体为 (消息个数)*2+2

serial::Serial ser;
float yaw_now,pitch_now,reboot_flag;
double PUB[6],pid_dt,pid_max,pid_min,pid_Kp,pid_Ki,pid_Kd,inte=0, pre_error=0;

void sendGimbal(int move1 , int move2 ,int yaw, int pitch,bool flag1,bool flag2)
{
    uint8_t sent[sendlength];
    union Buffer{
        short int data[(sendlength-2)/2];
        char char_data[(sendlength-2)];
    }buffer2;
//*******************对发出数据的赋值
   // buffer2.data[0] = move1;
    //buffer2.data[1] = move2;
    buffer2.data[0] = yaw;
    buffer2.data[1] = pitch;
    buffer2.data[2] = flag1;
    buffer2.data[3] = flag2;
//*******************

    sent[0]=0x2a;
    sent[sendlength-1]=0x3b;
    for (int i = 1;i < sendlength-1;i++)
    {
        sent[i] = buffer2.char_data[i-1];
    }
    //ser.write(sent, sendlength);
}

double PID( double dt, double max, double min, double Kp, double Ki, double Kd, double aim, double state_now, double & inte, double & pre_error )
{
    
    // Calculate error
    double error = aim - state_now;

    // Proportional term
    double Pout = Kp * error;

    // Integral term
    inte += error * dt;
    double Iout = Ki * inte;
    if( Iout > 5 )
        Iout = 5;
    // Derivative term
    double derivative = (error - pre_error) / dt;
    double Dout = Kd * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;
    // Restrict to max/min
    if( output > max )
        output = max;
    else if( output < min )
        output = min;

    // Save error to previous error
    pre_error = error;

    return output;
}

void PUSH(bool flagg,double pitch_push,double yaw_push,bool flag1,bool flag2)
{
    float yaw, pitch;
    yaw = (yaw_push + yaw_now) * 100;
    pitch = (-pitch_push +  pitch_now) * 100;
    if(flagg == 1)    inte = 0;
    double pid_yaw = PID(pid_dt, pid_max, pid_min, pid_Kp, pid_Ki, pid_Kd, yaw/100, yaw_now, inte, pre_error );

    sendGimbal(0,0,pid_yaw*100+yaw, pitch,flag1,flag2);
    cout<<"yaw_order:"<<pid_yaw+yaw/100<<"  pitch_order:"<<pitch/100<<std::endl; 
}

int OpenSerial(){
    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if (ser.isOpen())
        ;//ROS_INFO_STREAM("Serial Port initialized.\n");
    else
        return -1;
}

double *  SerialMain()
{
       //std_msgs::Float64MultiArray pub_msg;//地面部队接收下位机数据：当前yaw、当前pitch、当前yaw角速度、当前pitch角速度、当前小车移动速度大小及方向
       //ros::Time start = ros::Time::now();
       union Buffer{
	    short int data[(receivelength-2)/2];
	    char char_data[(receivelength-2)];
	}buffer1;

        size_t n2 = ser.available();
	if(n2!=0)
	{
	    uint8_t buffer[1000];
	    ser.read(buffer,n2);
	    for(int i=0;i<receivelength;i++)
	    {
		if((buffer[i])==(0x2a) && (buffer[i+receivelength-1])==(0x3b))
		{
	    	    for(int j =0;j<(receivelength-2);j++)
			{
			    buffer1.char_data[j]=buffer[j+i+1];
			} 
///////////////////////////对接受数据的读取 
		/*pub_msg.data.push_back(buffer1.data[0] / 100.0 / 180 * 3.14);//yaw1
		pub_msg.data.push_back(buffer1.data[0] / 100.0 / 180 * 3.14);//pitch1
		//if(abs(buffer1.data[2]/100.0) < 0.1) pub_msg.data.push_back(0);
		 pub_msg.data.push_back(buffer1.data[2] / 100.0 / 180 * 3.14);//yaw1_velocity
		//if(abs(buffer1.data[3]/100.0) < 0.1) pub_msg.data.push_back(0);
		 pub_msg.data.push_back(buffer1.data[3] / 100.0 / 180 * 3.14);//pitch1_velocity
		pub_msg.data.push_back(buffer1.data[4] / 100.0);//car_speed
		pub_msg.data.push_back(buffer1.data[5] / 100.0);//theta_of_car_speed*/
///////////////////////////
		yaw_now = buffer1.data[0] / 100.0;
		pitch_now = buffer1.data[1] / 100.0;
        flag1 = buffer1.data[2];
        flag2 = buffer1.data[3];
        reboot_flag=buffer1.data[4];
		//pub_control.publish(pub_msg);
	        cout<<"current_yaw:"<<yaw_now<<endl;
	        cout<<"current_pitch:"<<pitch_now<<endl;
	/*        cout<<"33:"<<buffer1.data[2] / 100.0<<endl;
	        cout<<"44:"<<buffer1.data[3] / 100.0<<endl;
	        cout<<"55:"<<pub_msg.data.at(4)<<endl;
	        cout<<"66:"<<pub_msg.data.at(5)<<endl;*/
		}
	    }
	    cout<<endl;
	}
/*	
	ros::Time now = ros::Time::now();
	ros::Duration aaa = now - start;
	double Hz = 1.0/aaa.toSec() ; 
	cout << Hz << "*************";
*/

	for(int j=0;j < 6; j++){
		PUB[j] = buffer1.data[j]/ 100.0 / 180 * 3.14;		
	}

	return PUB;
}

