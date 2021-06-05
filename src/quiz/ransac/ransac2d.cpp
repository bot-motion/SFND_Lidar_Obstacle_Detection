/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

bool same(pcl::PointXYZ p, pcl::PointXYZ q)
{
	return (p.x == q.x && p.y == q.y && p.z == q.z);
}


std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult, currentInliersResult;
	srand(time(NULL));
	
	pcl::PointXYZ p1, p2;

 	int maxIndex;
   	maxIndex = cloud->size(); 
   	srand(time(0));

	for (int iteration = 0; iteration <= maxIterations; iteration++)
	{

		// Randomly sample two points and fit line
		while (same(p1, p2)) 
		{
			p1 = cloud->points[rand() % maxIndex];
			p2 = cloud->points[rand() % maxIndex];
		}

		double A = p1.y - p2.y;
		double B = p2.x - p1.x;
		double C = p1.x * p2.y - p2.x * p1.y;

		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier

		int pointIndex;
		for (pointIndex = 0; pointIndex < maxIndex; pointIndex++)
		{
			pcl::PointXYZ p = cloud->at(pointIndex);
			double distPointToLine = abs(A * p.x + B * p.y + C)/sqrt(A*A + B*B);
			if (distPointToLine <= distanceTol) 
				currentInliersResult.insert(pointIndex);
		}

		if (currentInliersResult.size() > inliersResult.size())
			inliersResult = currentInliersResult;

	}
	
	return inliersResult;

}


std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult, currentInliersResult;
	srand(time(NULL));
	
	pcl::PointXYZ p1, p2, p3;

 	int maxIndex;
   	maxIndex = cloud->size(); 
   	srand(time(0));

	for (int iteration = 0; iteration <= maxIterations; iteration++)
	{

		// Randomly sample three points and fit plane
		while (same(p1, p2) || same(p1,p3) || same(p2,p3)) 
		{
			p1 = cloud->points[rand() % maxIndex];
			p2 = cloud->points[rand() % maxIndex];
			p3 = cloud->points[rand() % maxIndex];
		}

		double A = (p2.y - p1.y)*(p3.z - p1.z) - (p2.z - p1.z)*(p3.y - p1.y); // (y2−y1)(z3−z1)−(z2−z1)(y3−y1),
		double B = (p2.z - p1.z)*(p3.x - p1.x) - (p2.x - p1.x)*(p3.z - p1.z); //(z2−z1)(x3−x1)−(x2−x1)(z3−z1), 
		double C = (p2.x - p1.x)*(p3.y - p1.y) - (p2.y - p1.y)*(p3.x - p1.x); //(x2−x1)(y3−y1)−(y2−y1)(x3−x1)
		double D = -(A*p1.x + B*p1.y + C*p1.z);

		// Measure distance between every point and fitted plane
		// If distance is smaller than threshold count it as inlier

		int pointIndex;
		for (pointIndex = 0; pointIndex < maxIndex; pointIndex++)
		{
			pcl::PointXYZ p = cloud->at(pointIndex);
			double distPointToPlane = abs(A * p.x + B * p.y + C * p.z + D)/sqrt(A*A + B*B + C*C);
			if (distPointToPlane <= distanceTol) 
				currentInliersResult.insert(pointIndex);
		}

		if (currentInliersResult.size() > inliersResult.size())
			inliersResult = currentInliersResult;

	}
	
	return inliersResult;

}



int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	
	std::unordered_set<int> inliers = RansacPlane(cloud, 100, 1);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
