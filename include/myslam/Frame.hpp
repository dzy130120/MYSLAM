#pragma once
#include<myslam/CommonInclude.hpp>

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
    SE3* T_c_w;

    FRAME(double Time_Stamp, Mat Img_Rgb, Mat Img_Depth);
    FRAME(){};
    ~FRAME(){};
    // find the depth in depth map
    double findDepth( const cv::KeyPoint& kp );

//    // Get Camera Center
//    Vector3d getCamCenter() const;

    void setPose( const SE3& T_c_w );

//    // check if a point is in this frame
//    bool isInFrame( const Vector3d& pt_world );

};

}
