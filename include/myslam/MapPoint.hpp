#pragma once
#include<myslam/CommonInclude.hpp>

namespace myslam
{
class MAPPOINT
{
public:
    typedef std::shared_ptr<MAPPOINT> Ptr;
	Vector3d    Pos;       // Position in world
    Vector3d    Norm;      // Normal of viewing direction 
    Mat         Descriptor; // Descriptor for matching 
    
    int         MatchedTimes;     // being an inliner in pose estimation
    int         VisibleTimes;     // being visible in current frame

    MAPPOINT(Vector3d pos);
    MAPPOINT(double x, double y, double z);
    MAPPOINT(Vector3d pos, Mat descriptor);
    MAPPOINT(double x, double y, double z, Mat descriptor);
    MAPPOINT(){};
    ~MAPPOINT(){};

};

}
