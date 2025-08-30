#ifndef RANSAC_H
#define RANSAC_H

#include "render.h"
#include <unordered_set>
#include "processPointClouds.h"

std::unordered_set<int> RansacPlane(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
    int maxIterations,
    float distanceTol);

std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr>
RansacThreedSegmant(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
    int maxIterations,
    float distancetol);

#endif // RANSAC_H
