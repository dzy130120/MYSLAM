#include<myslam/myslam.hpp>

namespace myslam
{
void EdgeProjectXYZ2UVPoseOnly::computeError()
{
    const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*> ( _vertices[0] );
    _error = _measurement - frame_->camera2pixel ( 
        pose->estimate().map(point_) );
}

void EdgeProjectXYZ2UVPoseOnly::linearizeOplus()
{
    g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap*> ( _vertices[0] );
    g2o::SE3Quat T ( pose->estimate() );
    Vector3d xyz_trans = T.map ( point_ );
    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];
    double z_2 = z*z;

    _jacobianOplusXi ( 0,0 ) =  x*y/z_2 *camera_->fx;
    _jacobianOplusXi ( 0,1 ) = - ( 1+ ( x*x/z_2 ) ) *camera_->fx;
    _jacobianOplusXi ( 0,2 ) = y/z * camera_->fx;
    _jacobianOplusXi ( 0,3 ) = -1./z * camera_->fx;
    _jacobianOplusXi ( 0,4 ) = 0;
    _jacobianOplusXi ( 0,5 ) = x/z_2 * camera_->fx;

    _jacobianOplusXi ( 1,0 ) = ( 1+y*y/z_2 ) *camera_->fy;
    _jacobianOplusXi ( 1,1 ) = -x*y/z_2 *camera_->fy;
    _jacobianOplusXi ( 1,2 ) = -x/z *camera_->fy;
    _jacobianOplusXi ( 1,3 ) = 0;
    _jacobianOplusXi ( 1,4 ) = -1./z *camera_->fy;
    _jacobianOplusXi ( 1,5 ) = y/z_2 *camera_->fy;
}



OPTIMIZATION::OPTIMIZATION( SE3 init_Pose, FRAME::Ptr frame_, CAMERA::Ptr camera_, cv::Mat inliers_, vector<cv::Point3f> MatchMapPoint_, vector<cv::Point2f> MatchKeyPoint_)
{

    linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    solver_ptr = new Block ( linearSolver );
    solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
    optimizer.setAlgorithm ( solver );
    pose = new g2o::VertexSE3Expmap();
    pose->setId ( 0 );
    pose->setEstimate ( g2o::SE3Quat ( init_Pose.rotation_matrix(), init_Pose.translation() ));
    optimizer.addVertex ( pose );
    for ( int i=0; i<inliers_.rows; i++ )
    {
        int index = inliers_.at<int> ( i,0 );
        // 3D -> 2D projection
        edge.push_back(new EdgeProjectXYZ2UVPoseOnly());
        edge[i]->setId ( i );
        edge[i]->setVertex ( 0, pose );
        edge[i]->frame_ = frame_;
        edge[i]->camera_ = camera_;
        edge[i]->point_ = Vector3d ( MatchMapPoint_[index].x, MatchMapPoint_[index].y, MatchMapPoint_[index].z );
        edge[i]->setMeasurement ( Vector2d ( MatchKeyPoint_[index].x, MatchKeyPoint_[index].y ) );
        edge[i]->setInformation ( Eigen::Matrix2d::Identity() );
        optimizer.addEdge ( edge[i] );

    }
    optimizer.initializeOptimization();
}

SE3 OPTIMIZATION::Optimization_run(int Iteration)
{
    optimizer.optimize ( Iteration );
    return SE3 ( pose->estimate().rotation(), pose->estimate().translation() );
}
/////////////////////////////////
OPTIMIZATIONLP::OPTIMIZATIONLP( CAMERA::Ptr camera_, MAP::Ptr Mp_)
{
    linearSolver = new g2o::LinearSolverCholmod<BalBlockSolver::PoseMatrixType>();
    dynamic_cast<g2o::LinearSolverCholmod<BalBlockSolver::PoseMatrixType>* >(linearSolver)->setBlockOrdering(true);  // AMD ordering , only needed for sparse cholesky solver
    solver_ptr = new BalBlockSolver(linearSolver);
    solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);
    optimizer.initializeOptimization();

}

void OPTIMIZATIONLP::Optimization_run(int Iteration)
{
    optimizer.optimize ( Iteration );

}
}

