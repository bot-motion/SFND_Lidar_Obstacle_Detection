/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include "../../render/box.h"
#include <chrono>
#include <string>
#include "kdtree.h"
#include <cstdlib>

// Arguments:
// window is the region to draw box around
// increase zoom to see more of the area
pcl::visualization::PCLVisualizer::Ptr initScene(Box window, int zoom)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, zoom, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);

  	viewer->addCube(window.x_min, window.x_max, window.y_min, window.y_max, 0, 0, 1, 1, 1, "window");
  	return viewer;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData(std::vector<struct Point> points)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	
  	for(int i = 0; i < points.size(); i++)
  	{
  		pcl::PointXYZ point;

  		point.x = points[i].coordinates[0];
  		point.y = points[i].coordinates[1];
		if (points[i].coordinates.size() == 3)
  			point.z = points[i].coordinates[2];

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}


void render2DTree(Node* node, pcl::visualization::PCLVisualizer::Ptr& viewer, Box window, int& iteration, uint depth=0)
{

	if(node!=NULL)
	{
		Box upperWindow = window;
		Box lowerWindow = window;
		// split on x axis
		if(depth%2==0)
		{
			viewer->addLine(pcl::PointXYZ(node->point.coordinates[0], window.y_min, 0),pcl::PointXYZ(node->point.coordinates[0], window.y_max, 0),0,0,1,"line"+std::to_string(iteration));
			lowerWindow.x_max = node->point.coordinates[0];
			upperWindow.x_min = node->point.coordinates[0];
		}
		// split on y axis
		else
		{
			viewer->addLine(pcl::PointXYZ(window.x_min, node->point.coordinates[1], 0),pcl::PointXYZ(window.x_max, node->point.coordinates[1], 0),1,0,0,"line"+std::to_string(iteration));
			lowerWindow.y_max = node->point.coordinates[1];
			upperWindow.y_min = node->point.coordinates[1];
		}
		iteration++;

		render2DTree(node->left,viewer, lowerWindow, iteration, depth+1);
		render2DTree(node->right,viewer, upperWindow, iteration, depth+1);


	}

}



void printClusters(std::vector<std::vector<int>> clusters)
{
	for (int clusterNum = 0; clusterNum < clusters.size(); clusterNum++)
	{
		std::vector<int> c = clusters.at(clusterNum);
		for (int idx : c)
			std::cout << "Cluster # " << clusterNum << " contains point idx " << idx << std::endl;
	}
}


std::vector<int> proximity(int idx, std::vector<struct Point>& points, std::vector<int> cluster, KdTree* tree, float distanceTol)
{
/*	 Proximity(point,cluster):
		mark point as processed
		add point to cluster
		nearby points = tree(point)
		Iterate through each nearby point
			If point has not been processed
				Proximity(cluster)
 */

	points[idx].processed = true; 
	cluster.push_back(idx); 	// add point to cluster
	std::vector<int> nearbyPoints = tree->search(points[idx], distanceTol);  // returns a vector of point IDs

	for (int id : nearbyPoints)
	{
		if (!points[id].processed)
		{
			cluster = proximity(id, points, cluster, tree, distanceTol);
		}

	}

	return cluster;
}


std::vector<std::vector<int>> euclideanCluster(std::vector<struct Point>& points, KdTree* tree, float distanceTol)
{

	/*
	EuclideanCluster():
		list of clusters 
		Iterate through each point
			If point has not been processed
				Create cluster
				Proximity(point, cluster)
				cluster add clusters
		return clusters */

	std::vector<std::vector<int>> clusters;
 
	// notice that due to the initialization of the points in main() the point ID is the position in the points vector.
	// the data structures here need some serious improvement, but for now we'll exploit the above fact.

	for (int id = 0; id < points.size(); id++)
	{
		if (!points[id].processed)
		{
			std::vector<int> cluster;
			cluster = proximity(id, points, cluster, tree, distanceTol);
			clusters.push_back(cluster);
 		}
	}

	for (struct Point p : points)
	{
		p.print();
	}

	printClusters(clusters);

	return clusters;

}

int main ()
{

	// Create viewer
	Box window;
  	window.x_min = -10;
  	window.x_max =  10;
  	window.y_min = -10;
  	window.y_max =  10;
  	window.z_min =   0;
  	window.z_max =   0;
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene(window, 25);

	// Create data
	int seed = 10;			/* random seed */
	int numPoints = 10;		/* how many points to generate for clustering */
	int dim = 3;			/* dimensionality of the clustering */
	int range = 10;			/* coordinate space */

	srand (seed);

	std::vector<struct Point> points;
	for (int i = 0; i < numPoints; i++)
	{
		struct Point p;
		for (int d = 0; i < dim; d++)
		{
			p.coordinates.push_back(rand() % range);
		}
		p.processed = false;
		p.id = i;
		points.push_back(p);
	}

	//std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3} };
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData(points);

	KdTree* tree = new KdTree;
  
    for (int i=0; i<points.size(); i++) 
	{
    	tree->insert(points[i],i); 
	}

  	int it = 0;
  	render2DTree(tree->root,viewer,window, it);
  
  	std::cout << "Test Search" << std::endl;
  	std::vector<int> nearby = tree->search(points.back(),range/3);
  	for(int index : nearby)
      std::cout << index << ",";
  	std::cout << std::endl;

  	// Time segmentation process
  	auto startTime = std::chrono::steady_clock::now();
  	
  	std::vector<std::vector<int>> clusters = euclideanCluster(points, tree, range/10);
  	
  	auto endTime = std::chrono::steady_clock::now();
  	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  	std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;

  	// Render clusters
  	int clusterId = 0;
	std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
  	for(std::vector<int> cluster : clusters)
  	{
  		pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
  		for(int indice: cluster)
  			clusterCloud->points.push_back(pcl::PointXYZ(points[indice].coordinates[0],points[indice].coordinates[1],0));
  		renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(clusterId),colors[clusterId%3]);
  		++clusterId;
  	}
  	if(clusters.size()==0)
  		renderPointCloud(viewer,cloud,"data");
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
