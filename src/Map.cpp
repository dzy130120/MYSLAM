#include<myslam/myslam.hpp>

namespace myslam
{

MAP::MAP(const MAP::Ptr C)
{
    PointCloud.swap(C->PointCloud);
    Keyframes.swap(C->Keyframes);
}

void MAP::opD()
{
    for(int i=0;i<PointCloud.size();i++)
    {
        PointCloud[i]->Pos(0,0)=0.0;
        PointCloud[i]->Pos(1,0)=0.0;
        PointCloud[i]->Pos(2,0)=0.0;

        cvWaitKey(100);
    }
}

void MAP::Clone(const Ptr C)
{
    PointCloud.swap(C->PointCloud);
    Keyframes.swap(C->Keyframes);
}

void MAP::insertMapPoint ( MAPPOINT::Ptr map_point )
{
    PointCloud.push_back(map_point);
}

void MAP::isinMap(FRAME::Ptr frame_)
{
    for(int i=0; i<frame_->MapPoints.size();i++)
    {
        frame_->MapPoints[i]->Pos = frame_->T_c_w.inverse() * frame_->MapPoints[i]->Pos;
        bool inthismap = false;
        for(int j=0; j<PointCloud.size(); j++)
        {
            Matrix<double,3, 1> tmp_dis(PointCloud[j]->Pos-frame_->MapPoints[i]->Pos);

            if(tmp_dis.norm()<1)
            {
                PointCloud[j]->oberseverID.push_back(Keyframes.size());//被当前关键帧看到，对应当前地图中关键帧容器的序号，查找快速
                inthismap = true;
                break;
            }
        }
        if(!inthismap)
        {   frame_->MapPoints[i]->oberseverID.push_back(Keyframes.size());
            insertMapPoint(frame_->MapPoints[i]);
        }


    }

}
}
