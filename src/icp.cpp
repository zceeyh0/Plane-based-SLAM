#include "icp.h"

void find_feature_matches ( const cv::Mat& img_1, const cv::Mat& img_2,
                            std::vector<cv::KeyPoint>& keypoints_1,
                            std::vector<cv::KeyPoint>& keypoints_2,
                            std::vector< cv::DMatch >& matches )
{
    cv::Mat descriptors_1, descriptors_2;
    // use this in OpenCV3
    // Ptr<cv::FeatureDetector> detector = ORB::create();
    // Ptr<cv::DescriptorExtractor> descriptor = ORB::create();
    // use this in OpenCV2
    cv::Ptr<cv::FeatureDetector> detector = cv::FeatureDetector::create ( "ORB" );
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::DescriptorExtractor::create ( "ORB" );
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    vector<cv::DMatch> match;
   // BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_1, descriptors_2, match );

    double min_dist=10000, max_dist=0;

    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = match[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( match[i].distance <= max ( 2*min_dist, 30.0 ) )
        {
            matches.push_back ( match[i] );
        }
    }
}

cv::Point2d pixel2cam ( const cv::Point2d& p, const cv::Mat& K )
{
    return cv::Point2d
           (
               ( p.x - K.at<double> ( 0,2 ) ) / K.at<double> ( 0,0 ),
               ( p.y - K.at<double> ( 1,2 ) ) / K.at<double> ( 1,1 )
           );
}

void pose_estimation_3d3d (
    const vector<cv::Point3f>& pts1,
    const vector<cv::Point3f>& pts2,
    cv::Mat& R, cv::Mat& t
)
{
    cv::Point3f p1, p2; // center of mass
    int N = pts1.size();
    for ( int i=0; i<N; i++ )
    {
        p1 += pts1[i];
        p2 += pts2[i];
    }
    p1 = cv::Point3f( cv::Vec3f(p1) / N);
    p2 = cv::Point3f( cv::Vec3f(p2) / N);
    vector<cv::Point3f> q1 (N), q2 (N); // remove the center
    for ( int i=0; i<N; i++ )
    {
        q1[i] = pts1[i] - p1;
        q2[i] = pts2[i] - p2;
    }

    // compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for ( int i=0; i<N; i++ )
    {
        W += Eigen::Vector3d ( q1[i].x, q1[i].y, q1[i].z ) * Eigen::Vector3d ( q2[i].x, q2[i].y, q2[i].z ).transpose();
    }
    //cout<<"W="<<W<<endl;

    // SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd ( W, Eigen::ComputeFullU|Eigen::ComputeFullV );
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    
    if (U.determinant() * V.determinant() < 0)
	{
        for (int x = 0; x < 3; ++x)
        {
            U(x, 2) *= -1;
        }
	}
    
    //cout<<"U="<<U<<endl;
    //cout<<"V="<<V<<endl;

    Eigen::Matrix3d R_ = U* ( V.transpose() );
    Eigen::Vector3d t_ = Eigen::Vector3d ( p1.x, p1.y, p1.z ) - R_ * Eigen::Vector3d ( p2.x, p2.y, p2.z );

    // convert to cv::Mat
    R = ( cv::Mat_<double> ( 3,3 ) <<
          R_ ( 0,0 ), R_ ( 0,1 ), R_ ( 0,2 ),
          R_ ( 1,0 ), R_ ( 1,1 ), R_ ( 1,2 ),
          R_ ( 2,0 ), R_ ( 2,1 ), R_ ( 2,2 )
        );
    t = ( cv::Mat_<double> ( 3,1 ) << t_ ( 0,0 ), t_ ( 1,0 ), t_ ( 2,0 ) );
}

// cvMat2Eigen
Eigen::Isometry3d cvMat2Eigen( cv::Mat& rvec, cv::Mat& tvec )
{
    Eigen::Matrix3d r;
    for ( int i = 0; i < 3; i++ )
        for ( int j = 0; j < 3; j++ )
            r(i,j) = rvec.at<double>(i,j);
    
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    
    Eigen::AngleAxisd angle(r);
    T = angle;
    T(0,3) = tvec.at<double>(0,0);
    T(1,3) = tvec.at<double>(1,0);
    T(2,3) = tvec.at<double>(2,0);
    return T;
    
    /*Eigen::Isometry3d T;
    for ( int i = 0; i < 3; i++ )
        for ( int j = 0; j < 3; j++ )
            T(i,j) = rvec.at<double>(i,j);
            
    T(0,3) = tvec.at<double>(0,0);
    T(1,3) = tvec.at<double>(1,0);
    T(2,3) = tvec.at<double>(2,0);
    return T;*/
}

Eigen::Matrix4f bundleAdjustment (
    const vector< cv::Point3f >& pts1,
    const vector< cv::Point3f >& pts2,
    Eigen::Isometry3d trans )
{
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;
    std::unique_ptr<Block::LinearSolverType> linearSolver ( new g2o::LinearSolverEigen<Block::PoseMatrixType>());
    std::unique_ptr<Block> solver_ptr ( new Block (std::move(linearSolver) ) );
    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( std::move(solver_ptr) );
    
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm( solver );

    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
    pose->setId(0);
    pose->setEstimate( g2o::SE3Quat(
        Eigen::Matrix3d::Identity(),
        Eigen::Vector3d( 0,0,0 )
    ) );
    optimizer.addVertex( pose );

    // edges
    int index = 1;
    vector<EdgeSE3RGBD*> edges;
    for ( size_t i=0; i<pts1.size(); i++ )
    {
        EdgeSE3RGBD* edge = new EdgeSE3RGBD(
            Eigen::Vector3d(pts2[i].x, pts2[i].y, pts2[i].z) );
        edge->setId( index );
        edge->setVertex( 0, dynamic_cast<g2o::VertexSE3Expmap*> (pose) );
        edge->setMeasurement( Eigen::Vector3d(
            pts1[i].x, pts1[i].y, pts1[i].z) );
        edge->setInformation( Eigen::Matrix3d::Identity()*1e4 );
        optimizer.addEdge(edge);
        index++;
        edges.push_back(edge);
    }

    // chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.setVerbose( true );
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    /*chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
    cout<<"optimization costs time: "<<time_used.count()<<" seconds."<<endl;*/

    cout<<endl<<"after optimization:"<<endl;
    trans = Eigen::Isometry3d( pose->estimate() );
    cout<<"Optimised T="<<endl<<trans.matrix()<<endl;
    Eigen::Matrix4f opT;
    for (int i = 0; i < 4; i++)
    {
    	for (int j = 0; j < 4; j++)
    	{
    	    opT(i,j) = -trans.matrix()(i,j);
    	}
    	opT(i,i) = -opT(i,i);
    }
    
    return opT;
}

