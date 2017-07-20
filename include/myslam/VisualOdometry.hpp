#pragma once
#include<myslam/CommonInclude.hpp>
#include<myslam/myslam.hpp>
#include<myslam/Map.hpp>
namespace myslam
{
class VISUALODOMETRY
{
public:
    typedef std::shared_ptr<VISUALODOMETRY> Ptr;
    VISUALODOMETRY(){};
    VISUALODOMETRY(CAMERA::Ptr camera_, FEATUREOPERATER::Ptr FeatureOperater_temp);
    ~VISUALODOMETRY(){};
    void addframe(FRAME::Ptr curr_temp);
    void predicmodel();
    void Matcher();
    void poseEstimation();
    bool isKeyFrame(FRAME::Ptr fm);
    void run();
    int state;
    FEATUREOPERATER::Ptr FeatureOperater_;
    CAMERA::Ptr camera_;
    FRAME::Ptr  KeyFrame_;       // reference key-frame
    FRAME::Ptr  ref_;
    FRAME::Ptr  curr_;      // current frame
    MAP::Ptr GlobalMap_;
    MAP::Ptr LocalMap_;
    vector<cv::Point3f>   MatchMapPoint;       // matched 3d points
    vector<cv::Point2f>   MatchKeyPoint;  // matched 2d pixels (index of kp_curr)
    cv::Ptr<BFMatcher>   MatcherFlann_;     // flann matcher
    vector<cv::DMatch> matches;
    Mat rvec, tvec, inliers;
};

}
