#include<myslam/myslam.hpp>

unsigned long myslam::FRAME::Frame_ID = 0;
void f1(myslam::VISUALODOMETRY::Ptr vo_)
{
    vo_->run();
}

int main()
{
    //读取配置文件
    myslam::Config::setParameterFile ( "../para.yaml" );
    //读取数据集目录
    string datedir = myslam::Config::get<string> ( "dataset_dir" );
    //读取相机内参
    double f_x = myslam::Config::get<double> ( "camera.fx" );
    double f_y = myslam::Config::get<double> ( "camera.fy" );
    double c_x = myslam::Config::get<double> ( "camera.cx" );
    double c_y = myslam::Config::get<double> ( "camera.cy" );
    //读取深度图尺度比例系数
    double depth_scale = myslam::Config::get<double> ( "camera.depth_scale" );
    //每帧初始特征点数
    int Num_Of_Features = myslam::Config::get<int> ( "number_of_features" );

    double Scale_Factor = myslam::Config::get<double> ( "scale_factor" );
    int Level_Pyramid = myslam::Config::get<int> ( "level_pyramid" );
    //定义相机对象
    myslam::CAMERA::Ptr camera_(new myslam::CAMERA(datedir,f_x, f_y, c_x, c_y, depth_scale));
    //定义局部地图
    myslam::MAP::Ptr LocalMap_(new myslam::MAP());
    myslam::MAP::Ptr lm(new myslam::MAP(LocalMap_));
    //定义特征操作对象（提取特征点和描述子）
    myslam::FEATUREOPERATER::Ptr FeatureOperater(new myslam::FEATUREOPERATER(Num_Of_Features, Scale_Factor, Level_Pyramid));
    //定义里程计对象
    myslam::VISUALODOMETRY::Ptr vo(new myslam::VISUALODOMETRY(camera_, FeatureOperater, LocalMap_));
    //建立视觉里程计线程
    myslam::THREAD<myslam::VISUALODOMETRY::Ptr, myslam::VISUALODOMETRY>::Ptr FrontEnd(new myslam::THREAD<myslam::VISUALODOMETRY::Ptr, myslam::VISUALODOMETRY>(vo, &myslam::VISUALODOMETRY::run));
    myslam::THREAD<myslam::MAP::Ptr, myslam::MAP>::Ptr mpop(new myslam::THREAD<myslam::MAP::Ptr, myslam::MAP>(lm, &myslam::MAP::opD));
//    vo->run();

    FrontEnd->t2->join();
    mpop->t2->join();
    return 0;
}
