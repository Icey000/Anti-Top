/*
*	@Author: Mountain
*	@Date:	 2020.04.13
*	@Brief:  This cpp file define the ArmorNumClassifier class, realize some function used
*/

#include"ArmorDetector.h"
#include<string>
int out_num=1;

ArmorNumClassifier::ArmorNumClassifier() {
	//svm = ml::SVM::create();
	svm = StatModel::load<SVM>("/home/bbtx2/standard_rm/src/zero/include/General/svm0528.xml");
	if (svm.empty())
	{
		cout << "Svm load error! Please check the path!" << endl;
		exit(0);
	}	

	armorImgSize = Size(40, 40);
	p = Mat();

	warpPerspective_mat = Mat(3, 3, CV_32FC1);
	dstPoints[0] = Point2f(0, 0);
	dstPoints[1] = Point2f(armorImgSize.width, 0);
	dstPoints[2] = Point2f(armorImgSize.width, armorImgSize.height);
	dstPoints[3] = Point2f(0, armorImgSize.height);
}

ArmorNumClassifier::~ArmorNumClassifier() {}




void ArmorNumClassifier::loadImg(Mat& srcImg) {

	//copy srcImg as warpPerspective_src
	(srcImg).copyTo(warpPerspective_src);

	//preprocess srcImg for the goal of acceleration
	cvtColor(warpPerspective_src, warpPerspective_src, 6);  //CV_BGR2GRAY=6
	threshold(warpPerspective_src, warpPerspective_src, 20, 255, THRESH_BINARY);
	//imshow("warpPerspective_src",warpPerspective_src);
}

void ArmorNumClassifier::getArmorImg(ArmorBox& armor) {
	//set the armor vertex as srcPoints
	for (int i = 0; i < 4; i++)
		srcPoints[i] = armor.armorVertices[i];

	//get the armor image using warpPerspective
	warpPerspective_mat = getPerspectiveTransform(srcPoints, dstPoints);  // get perspective transform matrix  Í¸Éä±ä»»¾ØÕó
	warpPerspective(warpPerspective_src, warpPerspective_dst, warpPerspective_mat, armorImgSize, INTER_NEAREST, BORDER_CONSTANT, Scalar(0)); //warpPerspective to get armorImage
	warpPerspective_dst.copyTo(armor.armorImg); //copyto armorImg
/*
	//-------------------------------------------------------------------------±£´æÍ¼Æ¬
	string out_name = "/home/bbtx2/a/" + to_string(out_num) + ".jpg";//Ðè¸ü»»ÎÄ¼þ¼Ð
	imwrite(out_name, armor.armorImg);
	out_num++;
	//----------------------------------------------------------------------
*/
}

void ArmorNumClassifier::setArmorNum(ArmorBox& armor) {

	// adapt armorImg to the SVM model sample-size requirement
	p = armor.armorImg.reshape(1, 1);
	p.convertTo(p, CV_32FC1);

	//set armor number according to the result of SVM  
	armor.armorNum = (int)svm->predict(p);
cout<<"aaaaaaaaaaaaaaaaaaaaaaaaaa";
	cout<<"ArmorNum:"<<armor.armorNum<<endl;
}
