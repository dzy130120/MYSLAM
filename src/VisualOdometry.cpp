#include<myslam/myslam.hpp>

namespace myslam
{
    VISUALODOMETRY::VISUALODOMETRY(CAMERA::Ptr camera_temp, FEATUREOPERATER::Ptr FeatureOperater_temp):camera_(camera_temp), FeatureOperater_(FeatureOperater_temp), ref_(nullptr), curr_(nullptr)
    {
        state = 0;
    }

    void VISUALODOMETRY::addframe(FRAME::Ptr curr_temp)
    {
        if(0 == state)
        {
            curr_ = curr_temp;
            ref_ = curr_;
            state = 1;
        }
        else if(1 == state)
        {
            ref_ = curr_;
            curr_ = curr_temp;


        }
    }

    void VISUALODOMETRY::run()
    {
        Mat outimg1, outimg2;
        for(int i=0; i<camera_->rgb_files.size(); i++)
        {
            FRAME::Ptr frame_(new myslam::FRAME(i, cv::imread(camera_->rgb_files[i],-1), cv::imread(camera_->depth_files[i])));
            FeatureOperater_->extractKeyPoints(frame_->ImgRgb, frame_->KeypointsCurr);
            FeatureOperater_->computeDescriptors(frame_->ImgRgb, frame_->KeypointsCurr, frame_->DescriptorsCurr);
            addframe(frame_);

            drawKeypoints( curr_->ImgRgb, curr_->KeypointsCurr, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
            drawKeypoints( ref_->ImgRgb, ref_->KeypointsCurr, outimg2, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
            imshow("keypoint1",outimg1);
            imshow("keypoint2",outimg2);
            cv::waitKey(20);
//            std::cout<<camera_->rgb_files[i]<<std::endl;
//            std::cout<<camera_->depth_files[i]<<std::endl;
        }
    }
}
