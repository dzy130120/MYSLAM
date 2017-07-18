#include<myslam/myslam.hpp>

namespace myslam
{
void MAP::insertMapPoint ( MAPPOINT::Ptr map_point )
{
    PointCloud.push_back(map_point);
}

bool MAP::isinMap(MAPPOINT::Ptr map_point)
{
    for(int i=0; i<PointCloud.size(); i++)
    {
        Matrix<double,3, 1> tmp_dis(PointCloud[i]->Pos-map_point->Pos);

        cout<<tmp_dis.norm()<<endl;

        if(tmp_dis.norm()<1)
        {
            return true;
        }
        else
        {
            continue;
        }
    }
    return false;
}
}
