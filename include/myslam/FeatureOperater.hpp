#pragma once
#include<myslam/CommonInclude.hpp>

namespace myslam
{
class FEATUREOPERATER
{
public:
    typedef std::shared_ptr<FEATUREOPERATER> Ptr;
	FEATUREOPERATER(){};
    FEATUREOPERATER(int Num_Of_Features, double Scale_Factor, int Level_Pyramid);
	~FEATUREOPERATER(){};
	cv::Ptr<cv::ORB> 		FeatureDetector_;  // orb detector and computer 
    vector<cv::KeyPoint>    KeypointsCurr;    // keypoints in current frame
    cv::Mat                 DescriptorsCurr;  // descriptor in current frame 
	int NumOfFeatures;   // number of features
    double ScaleFactor;   // scale in image pyramid
    int LevelPyramid;     // number of pyramid levels
    void extractKeyPoints(const Mat Img_Rgb_, vector<KeyPoint> &Keypoints_Curr_);
    void computeDescriptors(const Mat Img_Rgb_, vector<KeyPoint> &Keypoints_Curr_, Mat &Descriptors_Curr_);

};

}
