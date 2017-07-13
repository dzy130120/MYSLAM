#include<myslam/myslam.hpp>
unsigned long myslam::FRAME::Frame_ID = 0;
int main()
{

    myslam::Config::setParameterFile ( "../para.yaml" );
    string datedir = myslam::Config::get<string> ( "dataset_dir" );
    double f_x = myslam::Config::get<double> ( "camera.fx" );
    double f_y = myslam::Config::get<double> ( "camera.fy" );
    double c_x = myslam::Config::get<double> ( "camera.cx" );
    double c_y = myslam::Config::get<double> ( "camera.cy" );
    double depth_scale = myslam::Config::get<double> ( "camera.depth_scale" );
    int Num_Of_Features = myslam::Config::get<int> ( "number_of_features" );
    double Scale_Factor = myslam::Config::get<double> ( "scale_factor" );
    int Level_Pyramid = myslam::Config::get<int> ( "level_pyramid" );
    myslam::CAMERA::Ptr camera_(new myslam::CAMERA(datedir,f_x, f_y, c_x, c_y, depth_scale));
    camera_->rgb_files.pop_back();
    camera_->depth_files.pop_back();
    myslam::FEATUREOPERATER::Ptr FeatureOperater(new myslam::FEATUREOPERATER(Num_Of_Features, Scale_Factor, Level_Pyramid));

    myslam::VISUALODOMETRY::Ptr vo(new myslam::VISUALODOMETRY(camera_, FeatureOperater));
    vo->run();

    return 0;
}
