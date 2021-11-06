#include <vector>
#include <pcl/common/common.h>
#include "tree.h"

std::vector<int> proximity(int idx, std::vector<struct Point>& points, std::vector<int> cluster, KdTree* tree, float distanceTol)
{
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
	std::vector<std::vector<int>> clusters;
 
	for (int id = 0; id < points.size(); id++)
	{
		if (!points[id].processed)
		{
			std::vector<int> cluster;
			cluster = proximity(id, points, cluster, tree, distanceTol);
			clusters.push_back(cluster);
 		}
	}

	return clusters;

}


bool same(pcl::PointXYZ p, pcl::PointXYZ q)
{
	return (p.x == q.x && p.y == q.y && p.z == q.z);
}