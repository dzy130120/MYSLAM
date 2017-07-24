#pragma once
#include<myslam/CommonInclude.hpp>
namespace myslam
{

class CAMERA
{
public:
    typedef std::shared_ptr<CAMERA> Ptr;
    CAMERA(){};
    ~CAMERA(){};
    CAMERA(double c_x, double c_y, double f_x, double f_y, double depth_Scale);
    CAMERA(string data_path, double c_x, double c_y, double f_x, double f_y, double depth_Scale);

    double cx;
    double cy;
    double fx;
    double fy;
    SO3 K;
    double depthScale;
    vector<string> rgb_files, depth_files;
    vector<double> rgb_times, depth_times;//时间戳
};


}
