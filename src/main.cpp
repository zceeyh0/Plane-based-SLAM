#include "slamFunc.h"

typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > main_Block;

int main( int argc, char** argv )
{
    cout << "Initialising..." << endl;  
    std::unique_ptr<main_Block::LinearSolverType> main_linearSolver ( new g2o::LinearSolverEigen<main_Block::PoseMatrixType>());
    std::unique_ptr<main_Block> main_solver_ptr ( new main_Block (std::move(main_linearSolver) ) );
    g2o::OptimizationAlgorithmGaussNewton* main_solver = new g2o::OptimizationAlgorithmGaussNewton( std::move(main_solver_ptr) );
    g2o::SparseOptimizer main_optimizer;
    main_optimizer.setAlgorithm( main_solver );
    vector<g2o::VertexSE3Expmap*> poses;
    vector<EdgeSE3RGBD*> edges;
    int index = 0;
    cout << "Please choose your alignment method: " << endl;
    cout << "1: ICP\n" << "2: ICP Optimised\n" << "3: GICP\n" << endl;
    cin >> index;
    
    // the global point cloud is always visible
    pcl::visualization::CloudViewer viewer("viewer");
    //sleep(5);
    clock_t start = clock();
    int currIndex = 180, min = 181, max = 280;
    vector<FRAME> keyframes;
    FRAME lastFrame;
    // point cloud conversion
    readImage( lastFrame, currIndex );
    imageToPointCloud( lastFrame, currIndex );
    
    PointCloud::Ptr cloud (new PointCloud);
    cloud = lastFrame.itself;
    segRANSACandNormal( lastFrame );
    keyframes.push_back( lastFrame );
    
    // record camera trajectory in pose.txt
    ofstream poseFile("pose.txt", ios::out);
    if (!poseFile){
        poseFile.close();
        cout<<"error opening pose file"<<endl;
        return 0;
    }

    FRAME matchFrame;
    
    // read remaining frames and align with the last one
    for( currIndex = min; currIndex < max; currIndex++ )
    {
    	FRAME currFrame;
    	// point cloud conversion
        readImage( currFrame, currIndex );
        imageToPointCloud( currFrame, currIndex );
        
        // trial on multithreading technique but failed
        /*FRAME nextFrame;
        auto thread1 = [&nextFrame, currIndex] {
    	    readImage( nextFrame, currIndex + 1 );
            imageToPointCloud( nextFrame, currIndex + 1 );
    	    cout << currIndex + 1 << " run first!!!" << endl;
    	};
        thread task(thread1);
        task.detach();*/
        
        PointCloud::Ptr tmp (new PointCloud);
        tmp = currFrame.itself;
        segRANSACandNormal( currFrame );
        cout<<"source: "<<lastFrame.itself->points.size()<<endl;
        cout<<"target: "<<currFrame.itself->points.size()<<endl;
        
        // coarse alignment to filter out bad matches
        if( currIndex == min || matchFrame.index == 0 )
        {
            if( !coarseMatch(lastFrame, currFrame) )
            {
                cout<<"not ok"<<endl;
                matchFrame.index = 0;
                continue;
            }
            else
                matchFrame.index = 1;
        }
        else
        {
            if( !coarseMatch(matchFrame, currFrame) )
            {
                cout<<"not ok"<<endl;
                continue;
            }
        }
        matchFrame = currFrame;

        // three methods for fine alignment
	switch(index)
	{
	    case 1:
	    	ICP( lastFrame, currFrame );
	    	break;
	    case 2:
	    	ICPOptimised( lastFrame, currFrame );
	    	break;
	    case 3:
	    	GICP( lastFrame, currFrame );
	    	break;
	}
        
        // calculate trajectory
        Eigen::Matrix4f T;
        T = currFrame.trans;
        double qw, qx, qy, qz, tx, ty, tz;
        qw = sqrt( (T(0,0)+T(1,1)+T(2,2)) + 1.0 ) / 2.0; // trace of R
        qx = (T(1,2)-T(2,1)) / (4.0*qw);
        qy = (T(2,0)-T(0,2)) / (4.0*qw);
        qz = (T(0,1)-T(1,0)) / (4.0*qw);
        tx = T(0,3);
        ty = T(1,3);
        tz = T(2,3);
        poseFile<<tx<<" "<<ty<<" "<<tz<<" "<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<"\n";

        // loop closure detection
        if ( loopClosure ( currFrame, keyframes ) )
        {
            cout << "A loop has detected" << endl;
            continue;
        }
        else
            keyframes.push_back( currFrame );
        
        // global map modeling
        cloud = joinPointCloud( cloud, currFrame );
        currFrame.itself = cloud;
        viewer.showCloud( cloud );
        
        lastFrame = currFrame;
        cout<<"i: "<<currIndex<<endl;
        keyframes.push_back(currFrame);
    }
    cout << "keyframes: " << keyframes.size() << endl;

    pcl::io::savePCDFile( "result.pcd", *cloud );
    poseFile.close();
    
    clock_t end = clock();
    cout<<"total time: "<<(double)(end-start)/(double)CLOCKS_PER_SEC<<" s"<<endl;
    return 0;
}


