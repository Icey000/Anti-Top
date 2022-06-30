#include"ArmorDetector.h"
#include "ArmorNumClassifier.cpp"
/**
 * @brief 图像预处理部分：图像通道相减
 */
void ArmorDetector::rgbImgProcess(){    
    Mat grayImg = Mat::zeros( this->srcImg.size(), CV_8UC1 );
    this->binImg = Mat::zeros( this->srcImg.size(), CV_8UC1 );
    
    //1.通道分离并根据敌方颜色通道相减，得到灰度图
    std::vector<cv::Mat> channels;
    cv::split(this->srcImg, channels);
    
    if( this->enemyColor == RED ){
        grayImg = channels.at(2) - channels.at(0);
    }else{
        grayImg = channels.at(0) - channels.at(2);
    }
    
    //2.图像二值化
    threshold(grayImg, this->binImg, this->rgb_threshold, 255, THRESH_BINARY );
    
    //3.对二值图膨胀处理
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(3, 3)); //膨胀操作使用的掩膜
	dilate( this->binImg, this->binImg, kernel); //进行膨胀操作，使得灯条区域更加平滑有衔接
}

void ArmorDetector::showProcessedRgbImg(){
    imshow("ProcessedRgbImg", this->binImg);
}

/**
 * @brief 图像预处理部分：svm提取数字特征，仿射变换
 */
void ArmorDetector::tmpImgProcess(){
    imshow("ProcessedTmpImg", this->tmpImg);
}

/**
 * @brief 图像预处理部分：rgb+tmp
 */
void ArmorDetector::imgProcess( Mat & camera_src ){
    camera_src.copyTo(this->srcImg);
    this->rgbImgProcess();
    //this->tmpImgProcess();
    classifier.loadImg(this->srcImg);
}
