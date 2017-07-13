#include<myslam/myslam.hpp>
using namespace myslam;
CAMERA::CAMERA(double c_x, double c_y, double f_x, double f_y, double depth_Scale):cx(c_x),cy(c_y),fx(f_x),fy(f_y),depthScale(depth_Scale)
{
    //init kinect
}

CAMERA::CAMERA(string data_path, double c_x, double c_y, double f_x, double f_y, double depth_Scale):cx(c_x),cy(c_y),fx(f_x),fy(f_y),depthScale(depth_Scale)
{
    std::ifstream fin ( (data_path+"/associate.txt").c_str() );
    if ( !fin )
    {
        std::cout<<"please generate the associate file called associate.txt!"<<std::endl;
        return;
    }


    while ( !fin.eof() )
    {
        string rgb_time, rgb_file, depth_time, depth_file;
        fin>>rgb_time>>rgb_file>>depth_time>>depth_file;
        rgb_times.push_back ( atof ( rgb_time.c_str() ) );
        depth_times.push_back ( atof ( depth_time.c_str() ) );
        rgb_files.push_back ( data_path+"/"+rgb_file );
        depth_files.push_back ( data_path+"/"+depth_file );

        if ( fin.good() == false )
            break;
    }
    T_c_w = Eigen::Matrix4d::Identity(4,4);
    K <<fx, 0 , cx,
        0 , fy, cy,
        0 , 0 , 1 ;



}
