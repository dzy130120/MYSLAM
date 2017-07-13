#include<myslam/myslam.hpp>

namespace myslam
{
    FRAME::FRAME(double Time_Stamp, Mat Img_Rgb, Mat Img_Depth):TimeStamp(Time_Stamp),ImgRgb(Img_Rgb),ImgDepth(Img_Depth)
    {
        FrameID = Frame_ID++;
    }

    void FRAME::setPose( const SE3& T_c_w_ )
    {
        T_c_w = new SE3(T_c_w_);
    }


}
