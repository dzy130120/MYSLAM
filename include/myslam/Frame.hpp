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
    double DepthTimeStamp;
    double RGBTimeStamp;
    Mat ImgRgb;
    Mat ImgDepth;
    bool KeyFrame;
    vector<cv::KeyPoint>    KeypointsCurr;    // keypoints in current frame
    Mat                     DescriptorsCurr;  // descriptor in current frame
    SE3 T_c_w;
    CAMERA::Ptr camera_;
    vector<MAPPOINT::Ptr> MapPoints;
    FRAME(myslam::CAMERA::Ptr camera_temp, double RGBTimeStamp_,double DepthTimeStamp_, Mat Img_Rgb, Mat Img_Depth);
    FRAME(){};
    ~FRAME(){};
    // find the depth in depth map
    double findDepth( const cv::KeyPoint& kp );
    // remove the keypoints whose depth is 0
    void keypointfilterwithdepth();
//    // Get Camera Center
//    Vector3d getCamCenter() const;
    Vector3d pixel2camera ( const Vector2d& p_p, double depth );
    Vector3d world2camera ( const Vector3d& p_w, const SE3& T_c_w );
    Vector2d camera2pixel ( const Vector3d& p_c );
    void setPose(const SE3 T_c_w_ );
    void dispatch();
//    // check if a point is in this frame
    bool isInFrame( const Vector3d& pt_world );

};

}
