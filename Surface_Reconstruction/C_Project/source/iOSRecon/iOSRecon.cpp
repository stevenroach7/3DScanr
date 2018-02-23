#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/integral_image_normal.h>

using namespace pcl;
using namespace std;

PointXYZ pointFromString(std::string line);

PointCloud<PointNormal>::Ptr computeNormals(PointCloud<PointXYZ>::Ptr pointCloudPtr, PointXYZ viewpoint);

PointCloud<PointNormal>::Ptr generateNormalCloudFromFolder(string folderName);

PointCloud<PointNormal>::Ptr statisticFilter(PointCloud<PointNormal>::Ptr pointNormalCloudPtr);

PolygonMesh performSurfaceReconstruction(PointCloud<PointNormal>::Ptr inputPCLPointCloud);


int main()
{
	PointCloud<PointNormal>::Ptr allNormalPtr = generateNormalCloudFromFolder("input");

	PointCloud<PointNormal>::Ptr filteredNormalPtr = statisticFilter(allNormalPtr);

	PolygonMesh mesh = performSurfaceReconstruction(filteredNormalPtr);
	pcl::io::savePLYFile("mesh.ply", mesh);
	
	//pcl::PLYWriter test;
	//std::cout << test.generateHeaderASCII(*allNormalPtr) << endl;
	std::system("pause");

	return 0;
}

PointCloud<PointXYZ>::Ptr cloudFromString(string fileName) {
	std::ifstream textStream(fileName);
	PointCloud<PointXYZ>::Ptr cloudPtr(new PointCloud<PointXYZ>);
	// Writing temporary point cloud
	cloudPtr->width = 1000; // pre-allocating 1000 point slots
	cloudPtr->height = 1; // Always 1
	cloudPtr->is_dense = false;
	cloudPtr->points.resize(cloudPtr->width * cloudPtr->height);
	int count = 0;
	std::string line;
	getline(textStream, line); // skip viewpoint
	while (getline(textStream, line)) // looping through every line
	{
		cloudPtr->points[count] = pointFromString(line);
		count++;
	}
	// Fixing the size
	cloudPtr->width = count; // Always size of cloud
	cloudPtr->points.resize(cloudPtr->width * cloudPtr->height);
	cout << "# points: " << cloudPtr->size() << "      ";

	return cloudPtr;
}

PointCloud<PointNormal>::Ptr generateNormalCloudFromFolder(string folderName)
{
	PointCloud<PointNormal>::Ptr allNormalPtr(new PointCloud<PointNormal>); // output
	int frame = 0, skip = 0;
	// Looping through every file && STOP after 10 non-existed files 
	while (skip < 10)
	{
		string fileName = "input/Frame_index_" + to_string(frame) + ".txt";
		std::cout << fileName;
		ifstream txtFile(fileName);
		if (txtFile.is_open()) {
			skip = 0;
			// Set view point
			string firstLine;
			getline(txtFile, firstLine);
			PointXYZ viewPoint = pointFromString(firstLine);
			// Read pointcloud from txt 
			PointCloud<PointXYZ>::Ptr tempPointCloudPtr = cloudFromString(fileName);
			// Compute normals for pointcloud and add to the output cloud
			PointCloud<PointNormal>::Ptr tempPointCloudNormalsPtr = computeNormals(tempPointCloudPtr, viewPoint);
			cout << "inliers: " << tempPointCloudNormalsPtr->size();
			*allNormalPtr += *tempPointCloudNormalsPtr;
			frame++;
		}
		else {
			skip++;
			frame++;
		}
		cout << endl;
	}
	std::cout << "Finish reading from txt files" << endl;
	pcl::io::savePLYFileASCII("allNormals.ply", *allNormalPtr);
	return allNormalPtr;
}

PointXYZ pointFromString(std::string line)
{
	PointXYZ point;
	std::stringstream lineStream(line);
	while (lineStream.good())
	{
		std::string segment;
		int index = 0;
		while (getline(lineStream, segment, ';'))
		{
			if (index == 0) {
				point.x = stof(segment);
			}
			if (index == 1) {
				point.y = stof(segment);
			}
			if (index == 2) {
				point.z = stof(segment);
			}
			index++;
		}
	}
	return point;
}

PointCloud<PointNormal>::Ptr computeNormals(PointCloud<PointXYZ>::Ptr pointCloudPtr, PointXYZ viewpoint)
{
	search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>());

	// Filtering Statistically
	//cout << "Statistically Filtering points" << endl;
	StatisticalOutlierRemoval<PointXYZ> statFilter;
	statFilter.setInputCloud(pointCloudPtr);
	statFilter.setMeanK((pointCloudPtr->size()-1));
	statFilter.setStddevMulThresh(0.01); 
	PointCloud<PointXYZ>::Ptr filteredPointCloudPtr(new PointCloud<PointXYZ>);
	statFilter.filter(*filteredPointCloudPtr);

	// Normal estimation
	NormalEstimationOMP<PointXYZ, Normal> ne;	
	ne.setSearchMethod(tree);
	ne.setNumberOfThreads(8);
	ne.setInputCloud(filteredPointCloudPtr);
	ne.setKSearch(10);
	ne.setViewPoint(viewpoint.x, viewpoint.y, viewpoint.z);

	// Compute normals
	PointCloud<Normal>::Ptr cloudNormalsPtr(new PointCloud<Normal>());
	ne.compute(*cloudNormalsPtr);

	PointCloud<PointNormal>::Ptr PointCloudNormalsPtr(new PointCloud<PointNormal>());
	pcl::concatenateFields(*filteredPointCloudPtr, *cloudNormalsPtr, *PointCloudNormalsPtr);

	return PointCloudNormalsPtr;
}

PointCloud<PointNormal>::Ptr statisticFilter(PointCloud<PointNormal>::Ptr pointNormalCloudPtr) {
	cout << "Do you want to apply Statistical Filter (true/false)? ";
	bool answer;
	cin >> answer;
	if (answer) {
		cout << "Statistically Filtering points" << endl;
		StatisticalOutlierRemoval<PointNormal> statFilter;
		// Set parametres
		statFilter.setInputCloud(pointNormalCloudPtr);
		statFilter.setMeanK(100);
		statFilter.setStddevMulThresh(1); // 0.6 - 1.0
		// Start filtering
		PointCloud<PointNormal>::Ptr filteredPointCloudPtr(new PointCloud<PointNormal>);
		statFilter.filter(*filteredPointCloudPtr);
		cout << "Statistical points filtering complete -- Total inliers:  " << filteredPointCloudPtr->size() << endl;
		// Output
		pcl::io::savePLYFile("filteredNormals.ply", *filteredPointCloudPtr);
		return filteredPointCloudPtr;
	}
	else return pointNormalCloudPtr;
}

PolygonMesh performSurfaceReconstruction(PointCloud<PointNormal>::Ptr inputPCLPointCloudPtr) {

	PointCloud<PointNormal>::Ptr pointNormalCloudPtr = inputPCLPointCloudPtr;
	cout << "Loaded Point Cloud with normals -- Total Points: " << pointNormalCloudPtr->size() << endl;

	cout << "Begin poisson reconstruction" << endl;
	Poisson<PointNormal> poisson;
	poisson.setDepth(6);
	poisson.setInputCloud(pointNormalCloudPtr);
	poisson.setPointWeight(4);
	poisson.setSamplesPerNode(1.5);

	PolygonMesh mesh;
	poisson.reconstruct(mesh);
	cout << "Mesh number of polygons: " << mesh.polygons.size() << endl;
	cout << "Poisson reconstruction complete" << endl;

	return mesh;
}
