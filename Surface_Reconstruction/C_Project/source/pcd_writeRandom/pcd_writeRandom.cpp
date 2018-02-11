#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;

string  pcd_writeRandom()
{
	pcl::PointCloud<pcl::PointXYZ> cloud;
	string cloudString;
	// Fill in the cloud data
	cloud.width = 5;
	cloud.height = 1;
	cloud.is_dense = false;
	cloud.points.resize(cloud.width * cloud.height);

	for (size_t i = 0; i < cloud.points.size(); ++i)
	{
		cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloudString = to_string(cloud.points[i].x) + (string) " ";

		cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloudString = to_string(cloud.points[i].y) + (string) " ";

		cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
		cloudString = to_string(cloud.points[i].z) + (string) " \n";
	}

	return cloudString;
}