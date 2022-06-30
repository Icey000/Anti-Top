
#include <deque>


int unidentify_factor = 0;          //未识别到系数
int anti_top = 0;            //陀螺系数
int continue_identify = 0;   //持续识别系数

//const int size_threshold = 0;     //前后两帧装甲板长宽差值平均数的阈值
const int height_threshold = 0.2;   //前后两帧装甲板中心高度差阈值
const int weight_threshold_max = 250;   //前后两帧装甲板中心宽度差最大值
const int weight_threshold_min = 180;   //前后两帧装甲板中心宽度差最小值

const int unidentify_threshold = 5;     //未识别到系数阈值
const int continue_threshold = 5; //持续识别系数阈值
const int anti_top_threshold = 5; //陀螺系数阈值

static std::deque<ArmorBox> HistoryTargets; //历史目标装甲板，最新在头部。

bool AntiTop (const ArmorDetector &armorDetector)
{
    if (armorDetector.state == ARMOR_FOUND)
    {
        if(HistoryTargets.size() < 2)
        {
            return false;
        }
        else
        {
            ArmorBox target_box = HistoryTargets[0]; // 当前目标装甲板
            ArmorBox last_box = HistoryTargets[1];   //上次目标装甲板
            cv::Point2f center_target = target_box.center;
            cv::Point2f center_last = last_box.center;
            
            if(abs(center_target.y - center_last.y) < height_threshold)
            {
                if ((abs(center_target.x - center_last.x) < weight_threshold_max) && (abs(center_target.x - center_last.x) > weight_threshold_min)) 
                {
                    anti_top++;
                }
                else if (abs(center_target.x - center_last.x) < weight_threshold_min)
                {
                    continue_identify++;
                    if(continue_identify > continue_threshold)
                    {
                        unidentify_factor = 0;
                    }
                }
            } 
            else
                anti_top = 0;
        }
        
    }
    else
    {
        unidentify_factor++;
        if (unidentify_factor > unidentify_threshold)
        {
            anti_top = 0;
            continue_identify = 0;
        }
    }
    if (anti_top > anti_top_threshold)
        return true;
    else
        return false;
}

