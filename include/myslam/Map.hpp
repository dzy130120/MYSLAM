#pragma once
#include<myslam/CommonInclude.hpp>
#include<myslam/MapPoint.hpp>

namespace myslam
{
class MAP
{
public:
    typedef std::shared_ptr<MAP> Ptr;
    MAP(){};
    MAP(const Ptr C);
	~MAP(){};
    vector<MAPPOINT::Ptr >  PointCloud;
    void insertMapPoint ( MAPPOINT::Ptr map_point );
    bool isinMap(MAPPOINT::Ptr);
    void opD();
};

}
