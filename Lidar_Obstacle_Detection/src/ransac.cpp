#include "ransac.h"
std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// TODO: Fill in this function
	auto minPointsNum = 3;
	// For max iterations
	while (maxIterations--)
	{
		// get 3 random points
		std::unordered_set<int> inliers;
		while (inliers.size() < minPointsNum)
		{
			inliers.insert(rand() % cloud->points.size());
		}

		// Randomly sample subset and fit line
		float X1, X2, X3, Y1, Y2, Y3, Z1, Z2, Z3;

		auto iter = inliers.begin();

		X1 = cloud->points[*iter].x;
		Y1 = cloud->points[*iter].y;
		Z1 = cloud->points[*iter].z;
		iter++; // go to the secound point
		X2 = cloud->points[*iter].x;
		Y2 = cloud->points[*iter].y;
		Z2 = cloud->points[*iter].z;
		iter++; // go to the third point
		X3 = cloud->points[*iter].x;
		Y3 = cloud->points[*iter].y;
		Z3 = cloud->points[*iter].z;

		// Measure distance between every point and fitted line
		// Ax + By + Cz + D = 0
		// for point1 (x1, y1, z1), point2 (x2, y2, z2)and point3 (x3, y3, z3)
		// define v1 and v2 vectors to get the a ,b , c
		pcl::PointXYZ v1(X2 - X1, Y2 - Y1, Z2 - Z1); // travels from point1 to point 2
		pcl::PointXYZ v2(X3 - X1, Y3 - Y1, Z3 - Z1); // travels from point1 to point 3

		// taking cross product of the vectors
		float a = v1.y * v2.z - v1.z * v2.y;
		float b = v1.z * v2.x - v1.x * v2.z;
		float c = v1.x * v2.y - v1.y * v2.x;
		float d = -(a * X1 + b * Y1 + c * Z1);

		// get random point to get x and y
		for (int index = 0; index < cloud->points.size(); index++)
		{
			// check the set it conatin the point
			if (inliers.count(index) > 0)
				continue;
			auto point = cloud->points[index];
			auto x = point.x;
			auto y = point.y;
			auto z = point.z;
			// Measure the distance from the point to the plane:
			auto distance = fabs(a * x + b * y + c * z + d) / sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2));

			// If distance is smaller than threshold count it as inlier
			if (distance <= distanceTol)
				inliers.insert(index);
		}
		// get the maximum inliers.
		if (inliersResult.size() < inliers.size())
			inliersResult = inliers;
	}

	// Return indicies of inliers from fitted line with max inliers
	return inliersResult;
}

std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> RansacThreedSegmant(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distancetol)
{
	// segment using ransac
	std::unordered_set<int> inliers = RansacPlane(cloud, maxIterations, distancetol);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZI>());

	for (int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZI point = cloud->points[index];
		if (inliers.count(index))
			cloudOutliers->points.push_back(point);
		else
			cloudInliers->points.push_back(point);
	}
	return std::make_pair(cloudInliers, cloudOutliers);
}
