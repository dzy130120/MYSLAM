#include<myslam/myslam.hpp>

namespace myslam
{
    FEATUREOPERATER::FEATUREOPERATER(int Num_Of_Features, double Scale_Factor, int Level_Pyramid)
	{
       FeatureDetector_ = ORB::create();
	}
	
    void FEATUREOPERATER::extractKeyPoints(const cv::Mat Img_Rgb_, std::vector<KeyPoint>& Keypoints_Curr_)
	{
        FeatureDetector_->detect(Img_Rgb_, Keypoints_Curr_);
	}

    void FEATUREOPERATER::computeDescriptors(const Mat Img_Rgb_, vector<KeyPoint> &Keypoints_Curr_, Mat& Descriptors_Curr_)
	{
        FeatureDetector_->compute(Img_Rgb_, Keypoints_Curr_, Descriptors_Curr_);
	}
}
