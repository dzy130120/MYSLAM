#include<myslam/myslam.hpp>

namespace myslam
{
    MAPPOINT::MAPPOINT(Vector3d pos):Pos(pos)
	{

	}
    MAPPOINT::MAPPOINT(double x, double y, double z)
	{
        Pos << x, y, z;
	}

    MAPPOINT::MAPPOINT(Vector3d pos, Mat descriptor):Pos(pos), Descriptor(descriptor)
    {

    }
    MAPPOINT::MAPPOINT(double x, double y, double z, Mat descriptor):Descriptor(descriptor)
    {
        Pos << x, y, z;
    }

    void MAPPOINT::setDescriptor(Mat descriptor)
    {
        Descriptor = (descriptor);
    }
}
