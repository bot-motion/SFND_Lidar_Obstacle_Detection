#ifndef CLUSTER_H_
#define CLUSTER_H_


std::vector<int> proximity(int idx, std::vector<struct Point>& points, std::vector<int> cluster, KdTree* tree, float distanceTol);

std::vector<std::vector<int>> euclideanCluster(std::vector<struct Point>& points, KdTree* tree, float distanceTol);

#endif