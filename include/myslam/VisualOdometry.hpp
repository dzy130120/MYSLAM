#pragma once
#include<myslam/CommonInclude.hpp>
#include<myslam/myslam.hpp>
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
    //void FindMatchs(FRAME::Ptr  Ref_, FRAME::Ptr  Curr_, );
    void run();
    int state;
    FEATUREOPERATER::Ptr FeatureOperater_;
    CAMERA::Ptr camera_;
    FRAME::Ptr  ref_;       // reference key-frame
    FRAME::Ptr  curr_;      // current frame
};

}
