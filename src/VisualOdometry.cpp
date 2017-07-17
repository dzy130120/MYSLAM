#include<myslam/myslam.hpp>

namespace myslam
{
    VISUALODOMETRY::VISUALODOMETRY(CAMERA::Ptr camera_temp, FEATUREOPERATER::Ptr FeatureOperater_temp):camera_(camera_temp), FeatureOperater_(FeatureOperater_temp), ref_(nullptr), KeyFrame_(nullptr), curr_(nullptr)
    {
        state = 0;
    }

    void VISUALODOMETRY::addframe(FRAME::Ptr curr_temp)
    {
        if(0 == state)
        {
            curr_ = curr_temp;
            curr_->KeyFrame = true;
            KeyFrame_ = curr_;
            ref_ = curr_;
            state = 1;
        }
        else if(1 == state)
        {
            curr_ = curr_temp;
            Matcher();
            KeyFrame_ = curr_;
        }
    }

    void VISUALODOMETRY::run()
    {
        Mat outimg1, outimg2;
        for(int i=0; i<camera_->rgb_files.size(); i++)
        {
            FRAME::Ptr frame_(new myslam::FRAME(camera_, i, cv::imread(camera_->rgb_files[i],-1), cv::imread(camera_->depth_files[i])));
            FeatureOperater_->extractKeyPoints(frame_->ImgRgb, frame_->KeypointsCurr);
            frame_->keypointfilterwithdepth();
            FeatureOperater_->computeDescriptors(frame_->ImgRgb, frame_->KeypointsCurr, frame_->DescriptorsCurr);
            frame_->dispatch();

            addframe(frame_);
            predicmodel();
//            drawKeypoints( curr_->ImgRgb, curr_->KeypointsCurr, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
//            drawKeypoints( ref_->ImgRgb, ref_->KeypointsCurr, outimg2, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
//            imshow("keypoint1",outimg1);
//            imshow("keypoint2",outimg2);
//            cv::waitKey(20);
//            std::cout<<camera_->rgb_files[i]<<std::endl;
//            std::cout<<camera_->depth_files[i]<<std::endl;
        }
    }

    void VISUALODOMETRY::predicmodel()
    {
        curr_->T_c_w = ref_->T_c_w;
    }

    void VISUALODOMETRY::Matcher()
    {
        if(KeyFrame_->DescriptorsCurr.type()!=CV_32F)
        {
            KeyFrame_->DescriptorsCurr.convertTo(KeyFrame_->DescriptorsCurr, CV_32F);
        }

        if(curr_->DescriptorsCurr.type()!=CV_32F)
        {
            curr_->DescriptorsCurr.convertTo(curr_->DescriptorsCurr, CV_32F);
        }
        MatcherFlann_.match(KeyFrame_->DescriptorsCurr, curr_->DescriptorsCurr, matches);

        // select the best matches
        float min_dis = std::min_element ( matches.begin(), matches.end(), [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
            {
                return m1.distance < m2.distance;
            } )->distance;
        Mat OutImage1, OutImage2;
        drawMatches(KeyFrame_->ImgRgb, KeyFrame_->KeypointsCurr, curr_->ImgRgb, curr_->KeypointsCurr, matches, OutImage1);
        imshow("before filer", OutImage1);

        for ( vector<cv::DMatch>::iterator m=matches.begin();m!=matches.end();)
        {
            if ( m->distance < max<float> ( min_dis*2.0 , 70.0 ) )
            {
                m++;

            }
            else
            {
                matches.erase(m);

            }
        }
        drawMatches(KeyFrame_->ImgRgb, KeyFrame_->KeypointsCurr, curr_->ImgRgb, curr_->KeypointsCurr, matches, OutImage2);
        imshow("after filer", OutImage2);
        cvWaitKey( 0 );
    }
}
