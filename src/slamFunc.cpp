#include "slamFunc.h"
#include "pnp.h"
#include "icp.h"
#include "../DBow3/src/DBoW3.h"

//To change dataset: Modify names in readImage and loopClosure

bool cmp_plane( PLANE x, PLANE y )
{
    return x.plane_size > y.plane_size;
}

bool cmpd( double x, double y )
{
    return x > y;
}

bool cmpv( Eigen::Vector4f x, Eigen::Vector4f y )
{
    return x(3) > y(3);
}

// this function reads rgb and depth data from a pair of images and stores them in a new created FRAME
void readImage ( FRAME& frame, int index_num )
{
    cv::Mat color, depth;
    // 1. read images in predefined directory: "./folder/index.png"
    boost::format fmt( "./%s/%d.%s" ); // directory of images
    // 1.1 read rgb images
    color = cv::imread( (fmt%"data/rgb"%(index_num)%"png").str() );
    // 1.2 read depth images (-1 means to read the original depth images)
    depth = cv::imread( (fmt%"data/depth"%(index_num)%"png").str(), -1 );

    // 2. output a FRAME with rgb and depth information
    frame.rgb = color;
    frame.depth = depth;
    frame.index = index_num;
}

// this function converts a pair of images into a point cloud
void imageToPointCloud( FRAME& frame, int index )
{
    PointCloud::Ptr pointCloud( new PointCloud ); 
    for ( int v=0; v<frame.rgb.rows; v++ )
        for ( int u=0; u<frame.rgb.cols; u++ )
        {
            // 1. transform points from pixel coordinates to camera coordinates 
            // reading depth value
            unsigned int d = frame.depth.ptr<unsigned short> ( v )[u]; // depth value
            if ( d==0 ) continue; // if depth is zero there is no measurement
            Eigen::Vector3d point;
            point[2] = double(d)/depthScale; 
            point[0] = (u-cx)*point[2]/fx;
            point[1] = (v-cy)*point[2]/fy;
                
            // 2. transform the point from camera coordinates to world coordinates
            Eigen::Vector3d pointWorld = point;
            PointT p;
            // convert positional information on the x, y and z axes
            p.x = pointWorld[0];
            p.y = pointWorld[1];
            p.z = pointWorld[2];
                
            // 3. extract RGB information in BGR structure
            p.b = frame.rgb.data[ v*frame.rgb.step+u*frame.rgb.channels() ];
            p.g = frame.rgb.data[ v*frame.rgb.step+u*frame.rgb.channels()+1 ];
            p.r = frame.rgb.data[ v*frame.rgb.step+u*frame.rgb.channels()+2 ];
            pointCloud->points.push_back( p );
        }
    // 4. Voxel Grid filter for downsampling
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud (pointCloud);
    vg.setLeafSize (0.02f, 0.02f, 0.02f); //leaf size of 2 cm by default
    vg.filter (*pointCloud);
        
    // 5. resize the point cloud to avoid alignment error
    pointCloud->is_dense = false;
    pointCloud->height = 1;
    pointCloud->width = pointCloud->points.size();
    // 6. output a FRAME with converted point cloud and input data
    frame.itself = pointCloud;
}

// this function convert PointXYZRGBA to PointXYZ for comparison
pcl::PointCloud<pcl::PointXYZ>::Ptr rgbToXYZ ( PointCloud::Ptr input )
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud (new pcl::PointCloud<pcl::PointXYZ>);
    int size = input->points.size();
    // convert every point from XYZRGB point cloud into XYZ point in a new XYZ point cloud
    for (int i = 0; i < size; i++)
    {
        pcl::PointXYZ p;
        p.x = input->points[i].x;
        p.y = input->points[i].y;
        p.z = input->points[i].z; 
        xyzCloud->points.push_back(p);
    }
    xyzCloud->width = 1;
    xyzCloud->height = size;
    return xyzCloud;
}

// RANSAC plane fitting and normal estimation on each segmented plane
void segRANSACandNormal ( FRAME& input )
{
    clock_t start = clock();
    // 1. Remove points with x, y, or z equal to NaN
    vector<int> indices_src; // store indices of NaN points
    PointCloud::Ptr cloud (new PointCloud);
    cloud = input.itself;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices_src);
    
    PointCloud::Ptr add (new PointCloud);
    pcl::PointCloud<pcl::Normal>::Ptr add_normals(new pcl::PointCloud<pcl::Normal>);
  
    // 2. Create a plane model segmentation object and set up parameters 
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE); // segmentation model 
    seg.setMethodType (pcl::SAC_RANSAC); // RANSAC estimation method
    seg.setMaxIterations (100); // maximum number of iterations
    seg.setDistanceThreshold (0.02); // threshold of distances between points
    
    PointCloud::Ptr tmp (new PointCloud()); // cloud data to be segmented
    pcl::copyPointCloud(*cloud, *tmp);

    // 3. Segment the filtered point cloud iteratively
    int num_points = (int) cloud->points.size();
    int i = 0;
    vector<Eigen::Vector4f> pnormal;
    while (tmp->points.size () > 0.3 * num_points)
    {
        // 3.1 Segment the largest planar component from the remaining cloud
        seg.setInputCloud (tmp);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
          cout << "Could not estimate a planar model for the given dataset." << endl;
          break;
        }
        PLANE plane;

        // 3.2 Get the points of the planar surface and add to the global point cloud
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud (tmp);
        extract.setIndices (inliers);
        extract.setNegative (false);
        PointCloud::Ptr cloud_plane (new PointCloud);
        extract.filter (*cloud_plane);
        plane.plane_size = (int) cloud_plane->points.size();
        //*add += *cloud_plane;
        
        // 3.3 Normal Estimation   
        // convert PointXYZRGBA to PointXYZ for comparison
        pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud (new pcl::PointCloud<pcl::PointXYZ>);
        xyzCloud = rgbToXYZ( cloud_plane );
        
        // 3.4 compute centroid
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid (*xyzCloud, centroid);
        // unit test
        /*cout<<"Centroid: "<<endl;
        cout<<centroid(0)<<endl;
        cout<<centroid(1)<<endl;
        cout<<centroid(2)<<endl;*/
    
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_centered (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointXYZ searchPoint;
        searchPoint.x = centroid(0);
        searchPoint.y = centroid(1);
        searchPoint.z = centroid(2);
    
        // 3.5 K nearest neighbor search on centroid
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud (xyzCloud);
        int K = 20;
        vector<int> pointIdxNKNSearch(K);
        vector<float> pointNKNSquaredDistance(K);

        kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
        cloud_centered->insert(cloud_centered->begin(), searchPoint); // insert centroid
        // 3.6 insert nearest points of centroid into a point cloud
        for (int i = 0; i < pointIdxNKNSearch.size(); i++)
        {
            pcl::PointXYZ point;
            point.x = xyzCloud->points[pointIdxNKNSearch[i]].x;
            point.y = xyzCloud->points[pointIdxNKNSearch[i]].y;
            point.z = xyzCloud->points[pointIdxNKNSearch[i]].z;
            cloud_centered->insert(cloud_centered->begin() + i+1, point);
        }
    
        // 3.7 Estimate normal on the centroid
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n; // OMP
        // nearest point sets using kdTree
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_centered_normals (new pcl::PointCloud<pcl::Normal>);
        // set input cloud and method
        n.setSearchMethod(tree);
        n.setKSearch(20);
        tree->setInputCloud(cloud_centered);
        n.setInputCloud(cloud_centered);
        n.setViewPoint(0,0,0);
        n.compute(*cloud_centered_normals);
    
        // visualising plane models with normals
        /*pcl::visualization::PCLVisualizer viewer("PCL viewer");
        viewer.setBackgroundColor(0.0, 0.0, 0.0);
        viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(add, cloud_centered_normals);

        while(!viewer.wasStopped())
        {
          viewer.spinOnce();
        }*/
        
        Eigen::Vector4f pn;
        pn(0) = cloud_centered_normals->points[0].normal_x;
        pn(1) = cloud_centered_normals->points[0].normal_y;
        pn(2) = cloud_centered_normals->points[0].normal_z;
        pn(3) = cloud_plane->points.size();
        pnormal.push_back(pn);
                
        // 3.8 Remove inliers of this segmentation and segment the remaining cloud
        extract.setNegative (true);
        extract.filter (*tmp);
        input.planes.push_back(plane);
        i++;
        //if (i == 5)
            //break;
    }
    
    // visualise estimated normals on all planes
    /*pcl::PointCloud<pcl::PointXYZ>::Ptr xyz (new pcl::PointCloud<pcl::PointXYZ>);
        xyz = rgbToXYZ( add );
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n; // OMP
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    tree->setInputCloud(xyz);
    n.setInputCloud(xyz);
    n.setViewPoint(0,0,0);
    n.compute(*cloud_normals);
    pcl::visualization::PCLVisualizer viewer("PCL viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(xyz, cloud_normals);

    while(!viewer.wasStopped())
    {
      viewer.spinOnce();
    }*/
    
    // 4. Sort the number of planes, normals and sums from great to small
    sort( input.planes.begin(), input.planes.end(), cmp_plane );
    sort( pnormal.begin(), pnormal.end(), cmpv );

    // unit test
    /*for (int a = 0; a < input.planes.size(); a++)
    {
        cout<<"Plane: "<<input.planes[a].plane_size<<endl;
    }*/
    
    // 5. save planes and associated informations
    input.plane_normal = pnormal; // normals on each segmented plane
    //pcl::io::savePCDFile( "add.pcd", *add ); // save all segmented planes in a point cloud
    clock_t end = clock();
    cout<<"Plane fitting and normal estimation time: "<<(double)(end-start)/(double)CLOCKS_PER_SEC<<" s"<<endl; // show time taken for each plane fitting and normal estimation on one scan
}

// this function filtered out bad matches before alignment based on number of planes and directions of surface normals on each plane
// if it returns true, current frame will be considered for next alignment
// if it returns false, current frame will be abandoned and alignment step will be skipped until the next correct current frame is received
bool coarseMatch (FRAME& last, FRAME& curr)
{
    int num_plane;
    // set comparison time as the number of lesser planes
    if ( curr.planes.size() < last.planes.size() )
        num_plane = curr.planes.size();
    else
        num_plane = last.planes.size();
    for ( int i = 0; i < num_plane; i++ )
    {
        // too large difference in points of planes
        if ( abs( curr.planes[i].plane_size - last.planes[i].plane_size ) > 8000 )
        {
            cout<<"Too large difference in planes!"<<endl;
            return false;
        }
        // angles between x-axis and y-axis
        float xy1 = atan(float(abs(last.plane_normal[i](1)))/float(abs(last.plane_normal[i](0))))*180.0/M_PI;
        float xy2 = atan(float(abs(curr.plane_normal[i](1)))/float(abs(curr.plane_normal[i](0))))*180.0/M_PI;
        // angles between y-axis and z-axis
        float yz1 = atan(float(abs(last.plane_normal[i](1)))/float(abs(last.plane_normal[i](2))))*180.0/M_PI;
        float yz2 = atan(float(abs(curr.plane_normal[i](1)))/float(abs(curr.plane_normal[i](2))))*180.0/M_PI;
        // angles between z-axis and x-axis
        float zx1 = atan(float(abs(last.plane_normal[i](0)))/float(abs(last.plane_normal[i](2))))*180.0/M_PI;
        float zx2 = atan(float(abs(curr.plane_normal[i](0)))/float(abs(curr.plane_normal[i](2))))*180.0/M_PI;
        if ( abs( xy2 - xy1 ) > 45 )
        {
            if ( abs( yz2 - yz1 ) > 45 && abs( zx2 - zx1 ) > 45 )
            {
                cout<<"Too large difference in directions!"<<endl;
                return false;
            }
        }
        if ( abs( yz2 - yz1 ) > 45 )
        {
            if ( abs( xy2 - xy1 ) > 45 && abs( zx2 - zx1 ) > 45 )
            {
                cout<<i<<"Too large difference in directions!"<<endl;
                return false;
            }
        }
        if ( abs( zx2 - zx1 ) > 45 )
        {
            if ( abs( yz2 - yz1 ) > 45 && abs( xy2 - xy1 ) > 45 )
            {
                cout<<i<<"Too large difference in directions!"<<endl;
                return false;
            }
        }
        
    }
    return true;
}

void ICP ( FRAME& last, FRAME& curr )
{
    //1. Read and match key points in images
    cv::Mat color1 = last.rgb;
    cv::Mat color2 = curr.rgb;
    cv::Mat depth1 = last.depth;
    cv::Mat depth2 = curr.depth;
    vector<cv::KeyPoint> keypoints_1, keypoints_2;
    vector<cv::DMatch> matches;
    find_feature_matches ( color1, color2, keypoints_1, keypoints_2, matches );
    cout<<"There are "<<matches.size() <<" pairs of matched points"<<endl; // unit test

    cv::Mat K = ( cv::Mat_<double> ( 3,3 ) << 518.0, 0, 325.5, 0, 519.0, 253.5, 0, 0, 1 );
    vector<cv::Point3f> pts1, pts2;

    //2. Build relationship between each pair of matched points
    for ( cv::DMatch m:matches )
    {
        ushort d1 = depth1.ptr<unsigned short> ( int ( keypoints_1[m.queryIdx].pt.y ) ) [ int  ( keypoints_1[m.queryIdx].pt.x ) ];
        ushort d2 = depth2.ptr<unsigned short> ( int ( keypoints_2[m.trainIdx].pt.y ) ) [ int ( keypoints_2[m.trainIdx].pt.x ) ];
        if ( d1==0 || d2==0 ) // bad depth value
            continue;
        cv::Point2d p1 = pixel2cam ( keypoints_1[m.queryIdx].pt, K );
        cv::Point2d p2 = pixel2cam ( keypoints_2[m.trainIdx].pt, K );
        float dd1 = float ( d1 ) /5000.0;
        float dd2 = float ( d2 ) /5000.0;
        pts1.push_back ( cv::Point3f ( p1.x*dd1, p1.y*dd1, dd1 ) );
        pts2.push_back ( cv::Point3f ( p2.x*dd2, p2.y*dd2, dd2 ) );
    }

    // 3. Calculate and optimise transformation matrix
    cout<<"3d-3d pairs: "<<pts1.size() <<endl;
    cv::Mat R, t, Rinv, tinv;
    pose_estimation_3d3d ( pts1, pts2, R, t );
    Rinv = R.t();
    tinv = -R.t() *t;
    // unit test
    /*cout<<"ICP via SVD results: "<<endl;
    cout<<"R = "<<R<<endl;
    cout<<"t = "<<t<<endl;
    cout<<"R_inv = "<<Rinv<<endl;
    cout<<"t_inv = "<<tinv<<endl;*/
    Eigen::Isometry3d T;
    Eigen::Matrix4f Top;
    T = cvMat2Eigen( Rinv, tinv );
    cout<<"ICP T matrix: "<<endl<<T.matrix()<<endl;
    clock_t start = clock();
    Top = bundleAdjustment( pts1, pts2, T ); // back-end optimisation
    clock_t end = clock();
    cout<<"G2O time: "<<(double)(end-start)/(double)CLOCKS_PER_SEC<<" s"<<endl;
    curr.trans = Top; // output the final transformation matrix
}

void ICPOptimised ( FRAME& last, FRAME& curr )
{
    //1. Read and match key points in images
    cv::Mat img_1 = last.rgb;
    cv::Mat img_2 = curr.rgb;
    cv::Mat d1 = last.depth;
    //cv::Mat depth2 = curr.depth;
    vector<cv::KeyPoint> keypoints_1, keypoints_2;
    vector<cv::DMatch> matches;
    feature_matches_pnp ( img_1, img_2, keypoints_1, keypoints_2, matches );
    cout<<"There are "<<matches.size() <<" pairs of matched points"<<endl; // unit test
    //Mat d1 = imread ( argv[3], CV_LOAD_IMAGE_UNCHANGED );
    Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
    vector<Point3f> pts_3d;
    vector<Point2f> pts_2d;
    
    //2. Build relationship between each pair of matched points
    for ( DMatch m:matches )
    {
        ushort d = d1.ptr<unsigned short> (int ( keypoints_1[m.queryIdx].pt.y )) [ int ( keypoints_1[m.queryIdx].pt.x ) ];
        if ( d == 0 ) // bad depth
            continue;
        float dd = d/5000.0;
        Point2d p1 = pixel2cam_pnp ( keypoints_1[m.queryIdx].pt, K );
        pts_3d.push_back ( Point3f ( p1.x*dd, p1.y*dd, dd ) );
        pts_2d.push_back ( keypoints_2[m.trainIdx].pt );
    }

    cout<<"3d-2d pairs: "<<pts_3d.size() <<endl;

    // 3. Calculate and optimise transformation matrix
    Mat r, t;
    solvePnP ( pts_3d, pts_2d, K, Mat(), r, t, false );
    //Mat inliers;
    //cv::solvePnPRansac( pts_3d, pts_2d, K, cv::Mat(), r, t, false, 100, 1.0, 100, inliers );
    Mat R;
    cv::Rodrigues ( r, R ); // Rodrigues to convert rotation matrix

    Eigen::Isometry3d T;
    Eigen::Matrix4f Top;
    
    T = cvMat2Eigen( R, t );
    cout<<"Optimised T matrix: "<<endl<<T.matrix()<<endl;
    for (int i = 0; i < 4; i++)
    {
    	for (int j = 0; j < 4; j++)
    	{
    	    Top(i,j) = T.matrix()(i,j);
    	}
    }
    curr.trans = Top;
}

// this function implements SAC initial alignment (not yet) and ICP registration
void GICP ( FRAME& one, FRAME& two )
{
    clock_t start = clock();
    
    // 1. convert PointXYZRGBA to PointXYZ
    pcl::PointCloud<pcl::PointXYZ>::Ptr source (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target (new pcl::PointCloud<pcl::PointXYZ>);
    source = rgbToXYZ( one.itself );
    target = rgbToXYZ( two.itself );
    
    // 2. initialise GICP object
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
 
    // 3. speed up searching using KdTree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
    tree1->setInputCloud(source);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
    tree2->setInputCloud(target);
    gicp.setSearchMethodSource(tree1);
    gicp.setSearchMethodTarget(tree2);
 
    // 4. set GICP parameters
    gicp.setInputSource(source);  // source point cloud
    gicp.setInputTarget(target);  // target point cloud
    //gicp.setSourceCovariances(one_cov); // covariance matrix of source point cloud
    //gicp.setTargetCovariances(two_cov); // covariance matrix of target point cloud
    //gicp.setTransformationEpsilon(1e-10); // epsilon for SVD singular decomposition
    //gicp.setEuclideanFitnessEpsilon(0.001);// threshold of error for iteration
    //gicp.setMaximumIterations(100); // threshold of maximum iteration time
    // gicp.setUseReciprocalCorrespondences(true); // using reciprocal correspondence relationship
    //gicp.setMaxCorrespondenceDistance(100); // threshold for maximum correspondence
 
    // 5. calculate rigid transformation from alignment
    pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    gicp.align(*icp_cloud);
    cout<<"gicp size: "<<icp_cloud->points.size()<<endl;
 
    // 6. output calculated transformation matrix
    // cout << "\nGICP has converged, score is " << gicp.getFitnessScore() << endl;
    cout << "T：\n" << gicp.getFinalTransformation() << endl;
    two.trans = gicp.getFinalTransformation();
    clock_t end = clock();
    cout<<"GICP time: "<<(double)(end-start)/(double)CLOCKS_PER_SEC<<" s"<<endl;
}

// this function returns true if two similar images are found (a loop is detected)
bool loopClosure ( FRAME& curr, vector<FRAME>& frames )
{
    DBoW3::Vocabulary vocab("./vocabulary.yml.gz");
    if ( vocab.empty() )
    {
        cerr<<"Vocabulary does not exist."<<endl;
        return 1;
    }
    string path_c = "data/rgb/"+to_string(curr.index)+".png";
    cv::Mat image_c = cv::imread(path_c) ;
    vector<cv::Mat> images; 
    for ( int i=0; i<frames.size(); i++ )
    {
        string path1 = "data/rgb/"+to_string(frames[i].index)+".png";
        images.push_back( cv::imread(path1) );
    }
    
    // NOTE: in this case we are comparing images with a vocabulary generated by themselves, this may leed to overfitting.  
    // detect ORB features
    cout<<"detecting ORB features ... "<<endl;
    cv::Ptr<cv::FeatureDetector> detector = cv::FeatureDetector::create ( "ORB" );
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::DescriptorExtractor::create ( "ORB" );
    
    vector<cv::Mat> descriptors;
    vector<cv::KeyPoint> keypoints_c; 
    
    cv::Mat descriptor_c;
    detector->detect ( image_c, keypoints_c );
    descriptor->compute ( image_c, keypoints_c, descriptor_c );
        
    for ( cv::Mat& image:images )
    {
        vector<cv::KeyPoint> keypoints_1; 
        cv::Mat descriptor_1;
        detector->detect ( image, keypoints_1 );
        descriptor->compute ( image, keypoints_1, descriptor_1 );
        descriptors.push_back( descriptor_1 );
    }
    
    // compare the images directly or we can compare one image to a database 
    cout<<"comparing images with images "<<endl;
    DBoW3::BowVector v1;
    vocab.transform( descriptor_c, v1 );
    vector<double> scores;
    double closest_score;
    for ( int i=0; i<images.size(); i++ )
    {
        DBoW3::BowVector v2;
        vocab.transform( descriptors[i], v2 );
        double score = vocab.score(v1, v2);
        scores.push_back( score );
        if ( i == images.size() - 1 )
            closest_score = score;
    }
    sort( scores.begin(), scores.end(), cmpd );
    // for (int i = 0; i < scores.size(); i++)
    	// cout << "score: " << scores[i] << endl;
    if ( scores[0] > closest_score * 2 )
        return true;
    return false;
}

// this function joins two point clouds for global mapping
PointCloud::Ptr joinPointCloud( PointCloud::Ptr original, FRAME& newFrame )
{
    // 1. join the transformed previous point cloud onto point cloud in current frame
    PointCloud::Ptr output (new PointCloud());
    PointCloud::Ptr temp (new PointCloud());

    pcl::transformPointCloud ( *original, *output, newFrame.trans );

    *newFrame.itself += *output;
    *temp = *newFrame.itself;
    
    // 2. Voxel grid filter downsampling
    pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setLeafSize(0.02f, 0.02f, 0.02f);
    voxel_grid.setInputCloud(temp);
    PointCloud::Ptr result(new PointCloud);
    voxel_grid.filter(*result);
    std::cout << "down size from " << newFrame.itself->size() << "to" << result->size() << endl;
    return result;
}

