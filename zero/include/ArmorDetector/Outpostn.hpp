#include <deque>
#include <vector>
 

int Ounidentify = 0;        //未识别到系数

int O_continue_identify = 0;

//当Z在3100-3500间


const int Owide_threshold_max = 200; //前后两帧装甲板中心宽度差最大值

const int Owide_threshold_min = 80;  //前后两帧装甲板中心宽度差最小值

const double time_threshold =0.05 ;


// const int unidentify_threshold = 5; //未识别到系数阈值

// const int continue_threshold = 5;   //持续识别系数阈值

// const int anti_top_threshold = 4;   //陀螺系数阈值，超过即可判断为陀螺状态

// const int anti_top_max = 6;         //陀螺系数最大值

// const int continue_number_max = 12; //持续识别系数最大值

 

static std::deque<ArmorBox> Outpostn_History;  //历史目标装甲板，最新在头部。
static std::deque<double> History_hittime;

bool armorchange = false;                    //判断是否突然识别到两块装甲板

int changenumber = 0;                        //表示装甲板数目变化次数

int changenumber_L = 0;                      //存储上一次的changenumber

static double first_t = 0;
static const double period = 0.4167;
static double code_t1 =0,code_t2=0;

static bool flag1 = 0,flag2 = 0;    //flag1是下位机发上来的，flag2是表示要击打的

static int t_num = 3;

double past_t = 0;

vector<double> hx,hy;


 

ArmorBox hit_center;                 //击打中心

double H_average(const vector<double> &arr)
{
    double sum=0;
    for (unsigned i = 1; i < arr.size(); i++)
        sum += arr[i];
    return  sum / (arr.size()-1);//平均值
}

double H_standardDev(const vector<double> &arr,double average)
{
    double sum = 0,variance=0;
    for (unsigned i = 1; i < arr.size(); i++)
        sum += pow(arr[i] - average, 2);
    variance = sum/(arr.size()-1); // 方差
    return sqrt(variance);     //标准差
}

void H_Chi_squared_Test(vector<double> &arr,double in,double vector_size)
{
    double ave,SD,range_up,range_down;
    arr.push_back(in);
    ave=H_average(arr);
    SD=H_standardDev(arr,ave);
    //.253-60//.524-70%//.842-80%
    range_up=ave+0.842*SD/sqrt(arr.size()-1);
    range_down=ave-0.842*SD/sqrt(arr.size()-1);
    
    if((in>range_up)||(in<range_down))
    {
    	in=arr[0];
    }
    else 
    {
        cout<<"离群点"<<endl;
    }	
    arr[0]=in;

    if((arr.size()-1)>=vector_size)
	arr.erase(arr.begin()+1,arr.begin()+2);
}

 

void catcht(const ArmorDetector &armorDetector,bool flag)  //反陀螺判断函数

{

    if(flag == 0)
    {
        changenumber = 0;
        changenumber_L = 0;

        first_t =0;
        t_num = 1;

        return ;

    }

    if (armorDetector.state == ARMOR_FOUND)  //首先保证有识别到的装甲板

    {

        if (Outpostn_History.size() < 2)       //然后还要确保已经在识别到的装甲板队列里存有两个装甲板

        {

            return ;

        }

        else

        {

            ArmorBox target_box = Outpostn_History[0]; // 取识别到的装甲板队列里的前两个进行比较判断

            ArmorBox last_box = Outpostn_History[1];   //

            cv::Point2f center_target = target_box.center;

            cv::Point2f center_last = last_box.center;
 
            cout << "KuanDU:" << abs(center_target.x - center_last.x) << "GaoDU:" << abs(center_target.y - center_last.y) <<endl;

 

            if ((abs(center_target.x - center_last.x) < Owide_threshold_max) && (abs(center_target.x - center_last.x) > Owide_threshold_min))

            {  

              //针对两个装甲板中心距离在wide_threshold_max和wide_threshold_min之间的我们认为是前哨站运动到了状态2  

                    if (armorchange != true)        

                    {

                        //如果装甲板状态发生了变化，则armorchange发生了变化，则changenumber++

                        changenumber++;

                        armorchange = true;

                    }

                    O_continue_identify++;
                    if (O_continue_identify > continue_threshold)

                    {

                        //持续识别系数大于阈值，则未识别到系数归零

                        Ounidentify = 0;

                    }
                    
            }
            else if (abs(center_target.x - center_last.x) < Owide_threshold_min)

            {

                //针对两个装甲板中心距离小于wide_threshold_min的我们认为是前哨站运动到了状态1

                    if (armorchange == true)

                    {

                        //如果装甲板状态发生了变化，则armorchange发生了变化，则changenumber++

                        changenumber++;

                        armorchange = false;

                    }

                    O_continue_identify++;
                    if (O_continue_identify > continue_threshold)

                    {

                        //持续识别系数大于阈值，则未识别到系数归零

                        Ounidentify = 0;

                    }

            }


        }

    }

    else if (armorDetector.state == ARMOR_NOT_FOUND)

    {

        //如果未识别到装甲板，则未识别到系数++

        Ounidentify++;

        if (Ounidentify > unidentify_threshold)      //未识别到系数大于阈值则说明有一段时间都未识别，之前的数值全归零，重新开始

        {

            Ounidentify = 0;


            changenumber = 0;
            changenumber_L = 0;

            first_t =0;

            t_num = 1;

        }

    }

    cout<<"changernumbe_L:"<<changenumber_L<<endl;

    cout<<"changernumber:"<<changenumber<<endl;


    struct timeval tv_now;
   if(changenumber==1)
    {
       changenumber_L = changenumber;     
        gettimeofday(&tv_now,NULL);

        double dt = double(tv_now.tv_sec);
        double dut = double(tv_now.tv_usec);
       dut/=1000000;

        double t_now = dt+dut;
        
        first_t = t_now;
        //past_t = t_now;
        // History_hittime.emplace_front(t_now);
        // History_hittime.emplace_front(t_now+(period/6));
        // History_hittime.emplace_front(t_now+(period/3));
	    // cout<<"firstTime:"<<t_now<<endl;
    }

    //  if ((changenumber % 2 == 1) && (changenumber != changenumber_L))
    //  {
    //     changenumber_L = changenumber;   
    //     double period = 2.5;
    //     gettimeofday(&tv_now,NULL);

    //     double ddt = double(tv_now.tv_sec);
    //     double ddut = double(tv_now.tv_usec);
    //     ddut/=1000000;

	//     double tt_now = ddt+ddut;

	//     if((0.8-(tt_now-past_t))>0.2)
    //     {
    //         Ounidentify++;
    //     }
                 
	//     past_t = tt_now;

    //  }
       
   return ;
}

 

ArmorBox hit(const ArmorDetector &armorDetector)                               //输出击打的位置

{
    ArmorBox zero;
if(armorDetector.state == ARMOR_FOUND)
{
    if(Outpostn_History.size() < 2)
    {
        return zero;
    }
    else
    {
        ArmorBox target_box = Outpostn_History[0]; // 当前目标装甲板

        ArmorBox last_box = Outpostn_History[1];   //上次目标装甲板

        hit_center.type = target_box.type;

        cv::Point2f center_target = target_box.center;

        cv::Point2f center_last = last_box.center;

        if ((abs(center_target.x - center_last.x) < Owide_threshold_max) && (abs(center_target.x - center_last.x) > Owide_threshold_min))

        {

            //针对状态2，我们求他两个装甲板的中心作为击打位置

            if (hit_center.center != zero.center)          //如果上次hit_center不为0，则结合这次求的进行加权平均（参数待定）

            {
                //hit_center.center = 0.9*((center_target + center_last) / 2) + 0.1*hit_center.center;
                hit_center.center = ((center_target + center_last) / 2) ;
                for(int i = 0; i<4 ; i++)
                {
                    hit_center.armorVertices[i] = ((target_box.armorVertices[i]+last_box.armorVertices[i]) / 2) ;
                    //hit_center.armorVertices[i] = 0.9*((target_box.armorVertices[i]+last_box.armorVertices[i]) / 2) + 0.1*hit_center.armorVertices[i];
                } 


            }

            if (hit_center.center == zero.center)           //上次没有则直接取这次
            {

                hit_center.center = (center_target + center_last) / 2;

                for(int i = 0; i<4 ; i++)
                {

                    hit_center.armorVertices[i] = ((target_box.armorVertices[i]+last_box.armorVertices[i]) / 2);
                } 

            }

        }

        //卡方检验
		H_Chi_squared_Test(hx,hit_center.center.x,5);
		H_Chi_squared_Test(hy,hit_center.center.y,5);
        cout<<"hitXXXX:"<<hx[0]<<endl;
	    cout<<"hitYYYY:"<<hy[0]<<endl;
	
    }
}
    return hit_center;                      

}
