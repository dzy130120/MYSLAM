#include<myslam/myslam.hpp>

namespace myslam
{
    VISUALODOMETRY::VISUALODOMETRY(CAMERA::Ptr camera_temp, FEATUREOPERATER::Ptr FeatureOperater_temp):camera_(camera_temp), FeatureOperater_(FeatureOperater_temp), ref_(nullptr), KeyFrame_(nullptr), curr_(nullptr)
    {
        state = 0;
        MAP::Ptr tmp(new MAP());
        LocalMap_ = tmp;
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
            for(int i=0 ;i<KeyFrame_->MapPoints.size(); i++)
            {
                LocalMap_->insertMapPoint(KeyFrame_->MapPoints[i]);
            }
        }
        else if(1 == state)
        {
            curr_ = curr_temp;
            predicmodel();
            Matcher();
            poseEstimation();
            if(isKeyFrame(curr_))
            {
                OPTIMIZATION::Ptr Optimization(new OPTIMIZATION(curr_->T_c_w, curr_, curr_->camera_, inliers, MatchMapPoint, MatchKeyPoint));
                curr_->T_c_w = Optimization->Optimization_run(10);
                for(int i=0; i<curr_->MapPoints.size();i++)
                {
                    curr_->MapPoints[i]->Pos = curr_->T_c_w.inverse() * curr_->MapPoints[i]->Pos;
                    if(LocalMap_->isinMap(curr_->MapPoints[i]))
                    {
                        continue;
                    }
                    else
                    {
                        LocalMap_->insertMapPoint(curr_->MapPoints[i]);
                    }

                }
                //cout<<LocalMap_->PointCloud.size()<<endl;
                KeyFrame_ = curr_;

            }

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
            std::cout<<camera_->rgb_files[i]<<std::endl;
            std::cout<<camera_->depth_files[i]<<std::endl;
            addframe(frame_);

//            drawKeypoints( curr_->ImgRgb, curr_->KeypointsCurr, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
//            drawKeypoints( ref_->ImgRgb, ref_->KeypointsCurr, outimg2, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
//            imshow("keypoint1",outimg1);
//            imshow("keypoint2",outimg2);
//            cv::waitKey(20);

        }
    }

    void VISUALODOMETRY::predicmodel()
    {
        curr_->T_c_w = KeyFrame_->T_c_w;
    }

    void VISUALODOMETRY::Matcher()
    {
        vector<MAPPOINT::Ptr>  candidate_pt;
        Mat  candidate_des;
        for(int n=0;n<LocalMap_->PointCloud.size();n++)
        {

            if(curr_->isInFrame(LocalMap_->PointCloud[n]->Pos))
            {
                candidate_pt.push_back(LocalMap_->PointCloud[n]);
                candidate_des.push_back(LocalMap_->PointCloud[n]->Descriptor);
            }
        }

        MatcherFlann_ = (new BFMatcher(NORM_HAMMING));
        MatcherFlann_->match(candidate_des, curr_->DescriptorsCurr, matches);

        // select the best matches
        float min_dis = std::min_element ( matches.begin(), matches.end(), [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
            {
                return m1.distance < m2.distance;
            } )->distance;
//        Mat OutImage1, OutImage2;
//        drawMatches(KeyFrame_->ImgRgb, KeyFrame_->KeypointsCurr, curr_->ImgRgb, curr_->KeypointsCurr, matches, OutImage1);
//        imshow("before filer", OutImage1);
//        cout<<matches.size()<<"  ";
        for ( vector<cv::DMatch>::iterator m=matches.begin();m!=matches.end();)
        {
//            cout<<m->distance<<" ";
            if ( m->distance < max<float> ( min_dis*2.0 , 30 ) )
            {
                MatchMapPoint.push_back(candidate_pt[m->queryIdx]->getPositionCV());
                MatchKeyPoint.push_back(curr_->KeypointsCurr[m->trainIdx].pt);
                m++;
            }
            else
            {

                matches.erase(m);

            }
        }

//        cout<<matches.size()<<endl;
//        drawMatches(KeyFrame_->ImgRgb, KeyFrame_->KeypointsCurr, curr_->ImgRgb, curr_->KeypointsCurr, matches, OutImage2);
//        imshow("after filer", OutImage2);
//        cvWaitKey( 0 );
    }

    void VISUALODOMETRY::poseEstimation()
    {
        Mat K = ( cv::Mat_<double> ( 3,3 ) <<
                      camera_->fx, 0, camera_->cx,
                      0, camera_->fy, camera_->cy,
                      0,0,1
                    );
        cv::solvePnPRansac ( MatchMapPoint, MatchKeyPoint, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers );
        curr_->T_c_w = SE3 (
                                   SO3 ( rvec.at<double> ( 0,0 ), rvec.at<double> ( 1,0 ), rvec.at<double> ( 2,0 ) ),
                                   Vector3d ( tvec.at<double> ( 0,0 ), tvec.at<double> ( 1,0 ), tvec.at<double> ( 2,0 ) )
                               );

//        typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1,-1>> Block;
//        Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
//        Block* solver_ptr = new Block ( linearSolver );
//        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
//        g2o::SparseOptimizer optimizer;
//        optimizer.setAlgorithm ( solver );

//        g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
//        pose->setId ( 0 );
//        pose->setEstimate ( g2o::SE3Quat (
//                SO3 ( rvec.at<double> ( 0,0 ), rvec.at<double> ( 1,0 ), rvec.at<double> ( 2,0 ) ).matrix(), Vector3d ( tvec.at<double> ( 0,0 ), tvec.at<double> ( 1,0 ), tvec.at<double> ( 2,0 ) )
//            ));
//        optimizer.addVertex ( pose );

//        for ( int i=0; i<inliers.rows; i++ )
//        {
//            int index = inliers.at<int> ( i,0 );
//            // 3D -> 2D projection
//            EdgeProjectXYZ2UVPoseOnly* edge = new EdgeProjectXYZ2UVPoseOnly();
//            edge->setId ( i );
//            edge->setVertex ( 0, pose );
//            edge->frame_ = curr_;
//            edge->camera_ = curr_->camera_;
//            edge->point_ = Vector3d ( MatchMapPoint[index].x, MatchMapPoint[index].y, MatchMapPoint[index].z );
//            edge->setMeasurement ( Vector2d ( MatchKeyPoint[index].x, MatchKeyPoint[index].y ) );
//            edge->setInformation ( Eigen::Matrix2d::Identity() );
//            optimizer.addEdge ( edge );

//        }
//        optimizer.initializeOptimization();
//        optimizer.optimize ( 10 );
//        curr_->T_c_w = SE3 ( pose->estimate().rotation(), pose->estimate().translation() );
    }

    bool VISUALODOMETRY::isKeyFrame(FRAME::Ptr fm)
    {
        if(fm->T_c_w.matrix().norm()>2)
        {
            fm->KeyFrame = true;
            return true;
        }
        else
        {
            return false;
        }
    }
}
