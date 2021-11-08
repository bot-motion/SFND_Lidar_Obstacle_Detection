// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
//#include "tree.h"
//#include "cluster.h"



//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Create the filtering object
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered);
  
    typename pcl::PointCloud<PointT>::Ptr cloud_region(new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_filtered);
    region.filter(*cloud_region);

    /*
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for (int p : indices)
        inliers.push_back(p);

    */
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_filtered;

}






template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // Create two new point clouds, one cloud with obstacles and other with segmented plane
	pcl::ExtractIndices<PointT> extract;
	
    typename pcl::PointCloud<PointT>::Ptr cloud_road (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr cloud_obst (new pcl::PointCloud<PointT> ());

  	extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_road);
  
    extract.setNegative(true);
    extract.filter(*cloud_obst);
  
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_road, cloud_obst);
    return segResult;
}




template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
 
 
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg; // pcl::PointXYZ
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
 
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
 
    if (inliers->indices.size() == 0)
    {   
        std::cout <<  "no model" << std::endl;
    }
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentRansac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{

    auto startTime = std::chrono::steady_clock::now();
	
	std::unordered_set<int> inliersResult, currentInliersResult;
	srand(time(NULL));
	
 	int maxIndex;
   	maxIndex = cloud->size(); 
   	srand(time(0));

	for (int iteration = 0; iteration <= maxIterations; iteration++)
	{

		// Randomly sample three points and fit plane
        std::unordered_set<int> indices;
        while (indices.size() < 3) 
        {
            indices.insert(rand() % maxIndex);
        }
        
        auto itr = indices.begin();
        auto p1 = cloud->points[*itr++];
        auto p2 = cloud->points[*itr++];
        auto p3 = cloud->points[*itr];


		double A = (p2.y - p1.y)*(p3.z - p1.z) - (p2.z - p1.z)*(p3.y - p1.y); // (y2−y1)(z3−z1)−(z2−z1)(y3−y1),
		double B = (p2.z - p1.z)*(p3.x - p1.x) - (p2.x - p1.x)*(p3.z - p1.z); //(z2−z1)(x3−x1)−(x2−x1)(z3−z1), 
		double C = (p2.x - p1.x)*(p3.y - p1.y) - (p2.y - p1.y)*(p3.x - p1.x); //(x2−x1)(y3−y1)−(y2−y1)(x3−x1)
		double D = -(A*p1.x + B*p1.y + C*p1.z);

		// Measure distance between every point and fitted plane
		// If distance is smaller than threshold count it as inlier

		int pointIndex;
		for (pointIndex = 0; pointIndex < maxIndex; pointIndex++)
		{
			PointT p = cloud->at(pointIndex);
			double distPointToPlane = abs(A * p.x + B * p.y + C * p.z + D)/sqrt(A*A + B*B + C*C);
			if (distPointToPlane <= distanceThreshold) 
				currentInliersResult.insert(pointIndex);
		}

		if (currentInliersResult.size() > inliersResult.size())
			inliersResult = currentInliersResult;

	}
	  
    if (inliersResult.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }
  
    
    std::cout << "separating clouds ..." << std::endl;
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr obstacleCloud(new pcl::PointCloud<PointT>());
    
    for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
			planeCloud->points.push_back(point);
		else
			obstacleCloud->points.push_back(point);
	}  
      
   	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, planeCloud);
      
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}






template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;

    ec.setClusterTolerance (clusterTolerance); // 2cm
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (clusterIndices);


    for (pcl::PointIndices getIndices : clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for (int idx : getIndices.indices)
            cloudCluster->points.push_back(cloud->points[idx]);

        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringEuclidean(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    auto startTime = std::chrono::steady_clock::now();   
  
   	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;   	
  	KdTree* tree = new KdTree;
  
  	std::vector<struct Point> cloud2Vector;
  
    std::cout << "tree insertion... " << std::endl;
    for(int i = 0; i < cloud->points.size(); i++)
    {
      struct Point pt;
      pt.coordinates[0] = cloud->points[i].x;
      pt.coordinates[1] = cloud->points[i].y;
      pt.coordinates[2] = cloud->points[i].z;
      pt.processed = false;
      pt.id = i;

      tree->insert(pt, i); 
      cloud2Vector.push_back(pt);     
    }
	
   	std::vector<std::vector<int>> results;
    std::cout << "custom clustering...";
  	results = euclideanCluster(cloud2Vector, tree, clusterTolerance);
    std::cout << "done" << std::endl;

    for(auto indexResults : results)
    {
        //drop clusters below and above tresholds
      	if(indexResults.size() < minSize || indexResults.size() > maxSize) 
    	{
        	continue; 
    	}
        
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for(int index : indexResults)
        {
            cloudCluster->points.push_back (cloud->points[index]);
        }
        cloudCluster->width=cloudCluster->points.size();
        cloudCluster->height=1;
        cloudCluster->is_dense=true;
        
        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}






template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}