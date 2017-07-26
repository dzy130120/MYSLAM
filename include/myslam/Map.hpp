#pragma once
#include<myslam/CommonInclude.hpp>
#include<myslam/MapPoint.hpp>

namespace myslam
{
class MAP
{
public:
    typedef std::shared_ptr<MAP> Ptr;
    vector<FRAME::Ptr> Keyframes;
    MAP(){};
    MAP(const Ptr C);
    void Clone(const Ptr C);
	~MAP(){};
    vector<MAPPOINT::Ptr >  PointCloud;
    void insertMapPoint ( MAPPOINT::Ptr map_point );
    void isinMap(FRAME::Ptr frame_);
    void opD();
};

}
