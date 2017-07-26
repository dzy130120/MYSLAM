#pragma once
#include<myslam/CommonInclude.hpp>
namespace myslam
{

//////////////////////////////////////////////////////////////////
// math functions needed for rotation conversion.

// dot and cross production

template<typename T>
inline T DotProduct(const T x[3], const T y[3]) {
  return (x[0] * y[0] + x[1] * y[1] + x[2] * y[2]);
}

template<typename T>
inline void CrossProduct(const T x[3], const T y[3], T result[3]){
  result[0] = x[1] * y[2] - x[2] * y[1];
  result[1] = x[2] * y[0] - x[0] * y[2];
  result[2] = x[0] * y[1] - x[1] * y[0];
}


//////////////////////////////////////////////////////////////////

template<typename T>
inline void AngleAxisRotatePoint(const T angle_axis[3], const T pt[3], T result[3]) {
  const T theta2 = DotProduct(angle_axis, angle_axis);
  if (theta2 > T(std::numeric_limits<double>::epsilon())) {
    // Away from zero, use the rodriguez formula
    //
    //   result = pt costheta +
    //            (w x pt) * sintheta +
    //            w (w . pt) (1 - costheta)
    //
    // We want to be careful to only evaluate the square root if the
    // norm of the angle_axis vector is greater than zero. Otherwise
    // we get a division by zero.
    //
    const T theta = sqrt(theta2);
    const T costheta = cos(theta);
    const T sintheta = sin(theta);
    const T theta_inverse = 1.0 / theta;

    const T w[3] = { angle_axis[0] * theta_inverse,
                     angle_axis[1] * theta_inverse,
                     angle_axis[2] * theta_inverse };

    // Explicitly inlined evaluation of the cross product for
    // performance reasons.
    /*const T w_cross_pt[3] = { w[1] * pt[2] - w[2] * pt[1],
                              w[2] * pt[0] - w[0] * pt[2],
                              w[0] * pt[1] - w[1] * pt[0] };*/
    T w_cross_pt[3];
    CrossProduct(w, pt, w_cross_pt);


    const T tmp = DotProduct(w, pt) * (T(1.0) - costheta);
    //    (w[0] * pt[0] + w[1] * pt[1] + w[2] * pt[2]) * (T(1.0) - costheta);

    result[0] = pt[0] * costheta + w_cross_pt[0] * sintheta + w[0] * tmp;
    result[1] = pt[1] * costheta + w_cross_pt[1] * sintheta + w[1] * tmp;
    result[2] = pt[2] * costheta + w_cross_pt[2] * sintheta + w[2] * tmp;
  } else {
    // Near zero, the first order Taylor approximation of the rotation
    // matrix R corresponding to a vector w and angle w is
    //
    //   R = I + hat(w) * sin(theta)
    //
    // But sintheta ~ theta and theta * w = angle_axis, which gives us
    //
    //  R = I + hat(w)
    //
    // and actually performing multiplication with the point pt, gives us
    // R * pt = pt + w x pt.
    //
    // Switching to the Taylor expansion near zero provides meaningful
    // derivatives when evaluated using Jets.
    //
    // Explicitly inlined evaluation of the cross product for
    // performance reasons.
    /*const T w_cross_pt[3] = { angle_axis[1] * pt[2] - angle_axis[2] * pt[1],
                              angle_axis[2] * pt[0] - angle_axis[0] * pt[2],
                              angle_axis[0] * pt[1] - angle_axis[1] * pt[0] };*/
    T w_cross_pt[3];
    CrossProduct(angle_axis, pt, w_cross_pt);

    result[0] = pt[0] + w_cross_pt[0];
    result[1] = pt[1] + w_cross_pt[1];
    result[2] = pt[2] + w_cross_pt[2];
  }
}

// camera : 9 dims array with
// [0-2] : angle-axis rotation
// [3-5] : translateion
// [6-8] : camera parameter, [6] focal length, [7-8] second and forth order radial distortion
// point : 3D location.
// predictions : 2D predictions with center of the image plane.

template<typename T>
inline bool CamProjectionWithDistortion(const T* camera, const T* point, T* predictions){
    // Rodrigues' formula
    T p[3];
    AngleAxisRotatePoint(camera, point, p);
    // camera[3,4,5] are the translation
    p[0] += camera[3]; p[1] += camera[4]; p[2] += camera[5];

    // Compute the center fo distortion
    T xp = -p[0]/p[2];
    T yp = -p[1]/p[2];

    // Apply second and fourth order radial distortion
    const T& l1 = camera[7];
    const T& l2 = camera[8];

    T r2 = xp*xp + yp*yp;
    T distortion = T(1.0) + r2 * (l1 + l2 * r2);

    const T& focal = camera[6];
    predictions[0] = focal * distortion * xp;
    predictions[1] = focal * distortion * yp;

    return true;
}

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

//////////////////////////////////////////////
/// \brief The OPTIMIZATIONLP class
///
///
/// ////////////////////////////////
class VertexCameraBAL : public g2o::BaseVertex<9,Eigen::VectorXd>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexCameraBAL() {}

    virtual bool read ( std::istream& /*is*/ )
    {
        return false;
    }

    virtual bool write ( std::ostream& /*os*/ ) const
    {
        return false;
    }

    virtual void setToOriginImpl() {}

    virtual void oplusImpl ( const double* update )
    {
        Eigen::VectorXd::ConstMapType v ( update, VertexCameraBAL::Dimension );
        _estimate += v;
    }

};


class VertexPointBAL : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexPointBAL() {}

    virtual bool read ( std::istream& /*is*/ )
    {
        return false;
    }

    virtual bool write ( std::ostream& /*os*/ ) const
    {
        return false;
    }

    virtual void setToOriginImpl() {}

    virtual void oplusImpl ( const double* update )
    {
        Eigen::Vector3d::ConstMapType v ( update );
        _estimate += v;
    }
};

class EdgeObservationBAL : public g2o::BaseBinaryEdge<2, Eigen::Vector2d,VertexCameraBAL, VertexPointBAL>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeObservationBAL() {}

    virtual bool read ( std::istream& /*is*/ )
    {
        return false;
    }

    virtual bool write ( std::ostream& /*os*/ ) const
    {
        return false;
    }

    virtual void computeError() override   // The virtual function comes from the Edge base class. Must define if you use edge.
    {
        const VertexCameraBAL* cam = static_cast<const VertexCameraBAL*> ( vertex ( 0 ) );
        const VertexPointBAL* point = static_cast<const VertexPointBAL*> ( vertex ( 1 ) );
        ( *this ) ( cam->estimate().data(), point->estimate().data(), _error.data() );

    }

    template<typename T>
    bool operator() ( const T* camera, const T* point, T* residuals ) const
    {
        T predictions[2];
        CamProjectionWithDistortion ( camera, point, predictions );
        residuals[0] = predictions[0] - T ( measurement() ( 0 ) );
        residuals[1] = predictions[1] - T ( measurement() ( 1 ) );

        return true;
    }


    virtual void linearizeOplus() override
    {
        // use numeric Jacobians
        // BaseBinaryEdge<2, Vector2d, VertexCameraBAL, VertexPointBAL>::linearizeOplus();
        // return;

        // using autodiff from ceres. Otherwise, the system will use g2o numerical diff for Jacobians

        const VertexCameraBAL* cam = static_cast<const VertexCameraBAL*> ( vertex ( 0 ) );
        const VertexPointBAL* point = static_cast<const VertexPointBAL*> ( vertex ( 1 ) );
        typedef ceres::internal::AutoDiff<EdgeObservationBAL, double, VertexCameraBAL::Dimension, VertexPointBAL::Dimension> BalAutoDiff;

        Eigen::Matrix<double, Dimension, VertexCameraBAL::Dimension, Eigen::RowMajor> dError_dCamera;
        Eigen::Matrix<double, Dimension, VertexPointBAL::Dimension, Eigen::RowMajor> dError_dPoint;
        double *parameters[] = { const_cast<double*> ( cam->estimate().data() ), const_cast<double*> ( point->estimate().data() ) };
        double *jacobians[] = { dError_dCamera.data(), dError_dPoint.data() };
        double value[Dimension];
        bool diffState = BalAutoDiff::Differentiate ( *this, parameters, Dimension, value, jacobians );

        // copy over the Jacobians (convert row-major -> column-major)
        if ( diffState )
        {
            _jacobianOplusXi = dError_dCamera;
            _jacobianOplusXj = dError_dPoint;
        }
        else
        {
            assert ( 0 && "Error while differentiating" );
            _jacobianOplusXi.setZero();
            _jacobianOplusXi.setZero();
        }
    }
};


class OPTIMIZATIONLP
{
public:
    typedef std::shared_ptr<OPTIMIZATIONLP> Ptr;
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<9,3> > BalBlockSolver;
    OPTIMIZATIONLP(){};
    ~OPTIMIZATIONLP(){};
    OPTIMIZATIONLP( CAMERA::Ptr camera_, MAP::Ptr Mp_);
    void Optimization_run(int Iteration);

    g2o::LinearSolver<BalBlockSolver::PoseMatrixType>* linearSolver;
    BalBlockSolver* solver_ptr;
    g2o::OptimizationAlgorithmWithHessian* solver;
    g2o::SparseOptimizer optimizer;
    vector<VertexPointBAL*> Points;
    vector<VertexCameraBAL*> poses;
    vector<EdgeObservationBAL*> edge;
};


}
