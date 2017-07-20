#pragma once
#include<myslam/CommonInclude.hpp>
namespace myslam
{
class EdgeProjectXYZ2UVPoseOnly: public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap >
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    virtual void computeError();
    virtual void linearizeOplus();
    
    virtual bool read( std::istream& in ){}
    virtual bool write(std::ostream& os) const {};
    
    Vector3d point_;
    FRAME::Ptr frame_;
    CAMERA::Ptr camera_;
};

class OPTIMIZATION
{
public:
    typedef std::shared_ptr<OPTIMIZATION> Ptr;
    OPTIMIZATION(){};
    ~OPTIMIZATION(){};
    OPTIMIZATION(SE3 init_Pose, FRAME::Ptr frame_, CAMERA::Ptr camera_, cv::Mat inliers_, vector<cv::Point3f> MatchMapPoint_, vector<cv::Point2f> MatchKeyPoint_);
    SE3 Optimization_run(int Iteration);
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1,-1>> Block;
    Block::LinearSolverType* linearSolver;
    Block* solver_ptr;
    g2o::OptimizationAlgorithmLevenberg* solver;
    g2o::SparseOptimizer optimizer;
    g2o::VertexSE3Expmap* pose;
    vector<EdgeProjectXYZ2UVPoseOnly*> edge;
    int index;
};

}
