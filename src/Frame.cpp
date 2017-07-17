#include<myslam/myslam.hpp>

namespace myslam
{
    FRAME::FRAME(CAMERA::Ptr camera_temp, double Time_Stamp, Mat Img_Rgb, Mat Img_Depth):camera_(camera_temp), TimeStamp(Time_Stamp),ImgRgb(Img_Rgb),ImgDepth(Img_Depth)
    {
        FrameID = Frame_ID++;
    }

    void FRAME::setPose(const Sophus::SE3 T_c_w_ )
    {
        T_c_w = T_c_w_;
    }

    double FRAME::findDepth( const cv::KeyPoint& kp )
    {
        int x = cvRound(kp.pt.x);
        int y = cvRound(kp.pt.y);
        ushort d = ImgDepth.ptr<ushort>(y)[x];
        if ( d!=0 )
        {
            return double(d)/camera_->depthScale;
        }
        else
        {
            // check the nearby points
            int dx[4] = {-1,0,1,0};
            int dy[4] = {0,-1,0,1};
            for ( int i=0; i<4; i++ )
            {
                d = ImgDepth.ptr<ushort>( y+dy[i] )[x+dx[i]];
                if ( d!=0 )
                {
                    return double(d)/camera_->depthScale;
                }
            }
        }
        return -1.0;
    }

    void FRAME::keypointfilterwithdepth()
    {
        for(vector<cv::KeyPoint>::iterator itor=KeypointsCurr.begin();itor!=KeypointsCurr.end();)
        {
            double d = findDepth( *itor );
            if(-1.0 == d)
            {
                KeypointsCurr.erase(itor);
            }
            else
            {
                int x = cvRound(itor->pt.x);
                int y = cvRound(itor->pt.y);
                MAPPOINT::Ptr tmp_pt(new myslam::MAPPOINT( pixel2camera(Vector2d(x,y), d) ));
                MapPoints.push_back(tmp_pt);//camera Coordinate System points
                itor++;
            }
        }
    }

    Vector3d FRAME::pixel2camera ( const Vector2d& p_p, double depth )
    {
        return Vector3d ( ( p_p ( 0,0 )-camera_->cx ) *depth/camera_->fx,  ( p_p ( 1,0 )-camera_->cy ) *depth/camera_->fy, depth );
    }

    Vector3d FRAME::world2camera ( const Vector3d& p_w, const SE3& T_c_w )
    {
        return T_c_w*p_w;
    }

    Vector2d FRAME::camera2pixel ( const Vector3d& p_c )
    {
        return Vector2d ( camera_->fx * p_c ( 0,0 ) / p_c ( 2,0 ) + camera_->cx, camera_->fy * p_c ( 1,0 ) / p_c ( 2,0 ) + camera_->cy );
    }

    void FRAME::dispatch()
    {
        for(int i = 0; i<MapPoints.size(); i++)
        {
            MapPoints[i]->Descriptor = DescriptorsCurr.rowRange(i,i+1).clone();
        }
    }

    bool FRAME::isInFrame( const Vector3d& pt_world )
    {
        Vector3d p_cam = world2camera( pt_world, T_c_w );
        // cout<<"P_cam = "<<p_cam.transpose()<<endl;
        if ( p_cam(2,0)<0 )
        {
            return false;
        }
        Vector2d pixel = camera2pixel( p_cam );
        // cout<<"P_pixel = "<<pixel.transpose()<<endl<<endl;
        return (pixel(0,0)>0) && (pixel(1,0)>0) && (pixel(0,0)<ImgRgb.cols) && (pixel(1,0)<ImgRgb.rows);
    }

}
