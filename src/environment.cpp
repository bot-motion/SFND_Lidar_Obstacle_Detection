/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
//#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = true;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    double groundSlope = 0.0;
    Lidar* egoLidar = new Lidar(cars, groundSlope);

    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = egoLidar->scan();
    //renderRays(viewer, egoLidar->position, inputCloud);

    renderPointCloud(viewer, inputCloud, "ego cloud", Color(1,1,1));

    
    ProcessPointClouds<pcl::PointXYZ>* pclProcessorPtr; 
    pclProcessorPtr = new ProcessPointClouds<pcl::PointXYZ>();

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud 
                    =  pclProcessorPtr->SegmentRansac(inputCloud, 100, 0.2f);
    
    renderPointCloud(viewer,segmentCloud.first,"cloud_road",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"cloud_obst",Color(0,1,0));
    

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pclProcessorPtr->ClusteringEuclidean(segmentCloud.second, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pclProcessorPtr->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);

        Box box = pclProcessorPtr->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        
        ++clusterId;
    }

}




void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,  ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.22f, Eigen::Vector4f(-20, -6, -5, 1), Eigen::Vector4f(30, 7, 5, 1));

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud 
                    =  pointProcessorI->SegmentRansac(filterCloud, 100, 0.22f);
    
    //renderPointCloud(viewer,segmentCloud.first,"cloud_obst",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"cloud_plane",Color(0,1,0));
    
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->ClusteringEuclidean(segmentCloud.first, 0.38f, 15, 1500);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,1,1)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId % colors.size()]);

        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        
        ++clusterId;
    }


}



//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
 
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_2/");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    while (!viewer->wasStopped()) {
    	viewer->removeAllPointClouds();
    	viewer->removeAllShapes();

    	inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
    	cityBlock(viewer, pointProcessorI, inputCloudI);
    	streamIterator++;
    	if (streamIterator == stream.end()) {
    		streamIterator = stream.begin();
    	}
    	viewer->spinOnce();
    }

    //simpleHighway(viewer);
    //cityBlock(viewer);

    //while (!viewer->wasStopped ())
    //{
    //    viewer->spinOnce ();
    //} 
}