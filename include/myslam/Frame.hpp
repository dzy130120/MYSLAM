#pragma once
#include<myslam/CommonInclude.hpp>
#include<myslam/MapPoint.hpp>

namespace myslam
{
class FRAME
{
public:
    typedef std::shared_ptr<FRAME> Ptr;
    static unsigned long Frame_ID;
    unsigned long FrameID;
    double TimeStamp;
    Mat ImgRgb;
    Mat ImgDepth;
    bool KeyFrame;
    vector<cv::KeyPoint>    KeypointsCurr;    // keypoints in current frame
    Mat                     DescriptorsCurr;  // descriptor in current frame
    Isometry3d T_c_w;
    CAMERA::Ptr camera_;
    vector<MAPPOINT::Ptr> MapPoints;
    FRAME(myslam::CAMERA::Ptr camera_temp, double Time_Stamp, Mat Img_Rgb, Mat Img_Depth);
    FRAME(){};
    ~FRAME(){};
    // find the depth in depth map
    double findDepth( const cv::KeyPoint& kp );
    // remove the keypoints whose depth is 0
    void keypointfilterwithdepth();
//    // Get Camera Center
//    Vector3d getCamCenter() const;
    Vector3d pixel2camera ( const Vector2d& p_p, double depth );
    void setPose(const Isometry3d T_c_w_ );

//    // check if a point is in this frame
//    bool isInFrame( const Vector3d& pt_world );

};

}
