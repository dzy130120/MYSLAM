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
	~MAP(){};
    unordered_map<unsigned long, MAPPOINT::Ptr >  PointCloud;

};

}
