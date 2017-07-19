#pragma once
#include<iostream>
#include<fstream>
#include<string>
#include<memory>
#include<vector>

#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
//#include<opencv2/core/eigen.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/features2d/features2d.hpp>
#include<sophus/se3.h>
#include<sophus/so3.h>
#include<Eigen/Core>
#include<Eigen/Dense>
#include<Eigen/Geometry>
#include<g2o/core/base_vertex.h>
#include<g2o/core/base_unary_edge.h>
#include<g2o/core/block_solver.h>
#include<g2o/core/optimization_algorithm_levenberg.h>
#include<g2o/types/sba/types_six_dof_expmap.h>
#include<g2o/solvers/dense/linear_solver_dense.h>
#include<g2o/core/robust_kernel.h>
#include<g2o/core/robust_kernel_impl.h>

using namespace cv;
using namespace std;
using namespace Sophus;
using namespace Eigen;
using namespace g2o;
