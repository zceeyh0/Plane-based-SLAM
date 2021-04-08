# pragma once

// header files 
// C++ standard libraries
#include <iostream>
#include <fstream> // files
#include <algorithm> // sort
#include <assert.h> // assert
#include <memory> // new, delete
#include <vector> // vector
#include <time.h> // clock_t
#include <math.h> // M_PI
#include <string>
#include <thread>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>

//BOOST
#include <boost/format.hpp>  // for formatting strings
#include <boost/shared_ptr.hpp>

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>

//G2O
#include <g2o/types/slam3d/types_slam3d.h> // types of vertices
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_dogleg.h"
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>

#include "pnp.h"
#include "icp.h"
#include "../DBow3/src/DBoW3.h"

using namespace std;

// type definition
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
using MatricesVector =
       std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>>;
   
// declaration of struct PLANE    
struct PLANE
{
    //pcl::ModelCoefficients coff; //a,b,c,d coefficients
    //vector<cv::KeyPoint> kp; // keypoints
    //vector<cv::Point3f> kp_pos; // 3d position of keypoints
    //cv::Mat desp; // descriptor
    //cv::Mat image; // grayscale image with mask
    //cv::Mat mask;
    int plane_size;
};

// declaration of struct FRAME
struct FRAME
{
    cv::Mat rgb, depth; // rgb and depth data
    PointCloud::Ptr itself; // a pointer to the point cloud of this frame
    pcl::PointCloud<pcl::Normal>::Ptr its_normal;
    // pcl::PointCloud<pcl::Normal>::Ptr its_normal; // a pointer to normals
    int index; // an index of this frame
    vector<PLANE> planes; // all segmented planes
    vector<Eigen::Vector4f> plane_normal; // normals on each segmented plane
    Eigen::Matrix4f trans; // transformation from last frame in a matrix
};

bool cmp_plane( PLANE x, PLANE y );
bool cmpd( double x, double y );
bool cmpv( Eigen::Vector4f x, Eigen::Vector4f y );

// camera intrinsic parameters from NYU-V2
const double cx = 325.5;
const double cy = 253.5;
const double fx = 518.0;
const double fy = 519.0;
const double depthScale = 1000.0;

void readImage ( FRAME& frame, int index_num );
void imageToPointCloud( FRAME& frame, int index );
pcl::PointCloud<pcl::PointXYZ>::Ptr rgbToXYZ ( PointCloud::Ptr input );
void segRANSACandNormal ( FRAME& input );
bool coarseMatch (FRAME& last, FRAME& curr);
void ICP ( FRAME& last, FRAME& curr );
void ICPOptimised ( FRAME& last, FRAME& curr );
void GICP ( FRAME& one, FRAME& two );
bool loopClosure ( FRAME& curr, vector<FRAME>& frames );
PointCloud::Ptr joinPointCloud( PointCloud::Ptr original, FRAME& newFrame );

