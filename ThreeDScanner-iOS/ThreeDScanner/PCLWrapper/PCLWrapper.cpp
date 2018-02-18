//
//  PCLWrapper.cpp
//  ThreeDScanner
//
//  Created by Steven Roach on 2/9/18.
//  Copyright © 2018 Steven Roach. All rights reserved.
//

#include "PCLWrapper.hpp"
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
#include <pcl/features/integral_image_normal.h>

using namespace pcl;
using namespace std;


/*
 Helper method to compute normals.
 */
PointCloud<Normal>::Ptr computeNormals(PointCloud<PointXYZ>::Ptr pointCloudPtr)
{
    cout << "Begin normal estimation" << endl;
    search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>());
    NormalEstimationOMP<PointXYZ, Normal> ne;
    ne.setSearchMethod(tree);
    ne.setNumberOfThreads(8);
    ne.setInputCloud(pointCloudPtr);
    ne.setKSearch(100);
    // Compute the centroid of point cloud
    Eigen::Vector4f centroid;
    compute3DCentroid(*pointCloudPtr, centroid);
    ne.setViewPoint(centroid[0], centroid[1], centroid[2]);
    // Compute normals
    PointCloud<Normal>::Ptr cloudNormalsPtr(new PointCloud<Normal>());
    ne.compute(*cloudNormalsPtr);
    cout << "Normal estimation complete" << endl;
    
    // Reverse normals
    cout << "Reverse normals' direction" << endl;
    for (size_t i = 0; i < cloudNormalsPtr->size(); ++i) {
        cloudNormalsPtr->points[i].normal_x *= -1;
        cloudNormalsPtr->points[i].normal_y *= -1;
        cloudNormalsPtr->points[i].normal_z *= -1;
    }
    return cloudNormalsPtr;
}

PCLMesh performSurfaceReconstruction(PCLPointCloud inputPCLPointCloud) {
    
    // Convert PCLPointCloud to PointCloud<XYZ>
    PointCloud<PointXYZ>::Ptr pointCloudPtr(new PointCloud<PointXYZ>);

    pointCloudPtr->width    = inputPCLPointCloud.numPoints; // Always size of cloud
    pointCloudPtr->height   = 1; // Always 1
    pointCloudPtr->is_dense = false;
    pointCloudPtr->points.resize (pointCloudPtr->width * pointCloudPtr->height); // Need this line
    
    cout << "Converting to PCL point cloud" << endl;
    cout << "Num points = " << inputPCLPointCloud.numPoints << endl;

    for (size_t i = 0; i < inputPCLPointCloud.numPoints; i++)
    {
        pointCloudPtr->points[i].x = inputPCLPointCloud.points[i].x;
        pointCloudPtr->points[i].y = inputPCLPointCloud.points[i].y;
        pointCloudPtr->points[i].z = inputPCLPointCloud.points[i].z;
    }

    cout << "Loaded Point Cloud" << endl;
    
    cout << "Begin passthrough filter" << endl;
    PointCloud<PointXYZ>::Ptr filteredPointCloudPtr(new PointCloud<PointXYZ>());
    PassThrough<PointXYZ> filter;
    filter.setInputCloud(pointCloudPtr);
    filter.filter(*filteredPointCloudPtr);
    cout << "Passthrough filter complete" << endl;
    
    PointCloud<Normal>::Ptr pointCloudNormalsPtr = computeNormals(filteredPointCloudPtr);
    
    cout << "Combine points and normals" << endl;
    PointCloud<PointNormal>::Ptr cloudSmoothedNormalsPtr(new PointCloud<PointNormal>());
    concatenateFields(*filteredPointCloudPtr, *pointCloudNormalsPtr, *cloudSmoothedNormalsPtr);
    
    cout << "Begin poisson reconstruction" << endl;
    Poisson<PointNormal> poisson;
    poisson.setDepth(6);
    poisson.setInputCloud(cloudSmoothedNormalsPtr);
    poisson.setPointWeight(0);
    poisson.setSamplesPerNode(1);
    
    PolygonMesh mesh;
    poisson.reconstruct(mesh);
    
    cout << "Mesh number of polygons: " << mesh.polygons.size() << endl;
    cout << "Poisson reconstruction complete" << endl;
    
    // Convert to output format
    
    // Need mesh cloud in PointCloud<PointXYZ> format instead of PointCloud2
    PointCloud<PointXYZ> meshXYZPointCloud;
    fromPCLPointCloud2(mesh.cloud, meshXYZPointCloud);
    
    long int meshNumPoints = meshXYZPointCloud.size();
    long int meshNumFaces = mesh.polygons.size();
    
    PCLPoint3D *meshPointsPtr;
    meshPointsPtr = (PCLPoint3D *) calloc(meshNumPoints, sizeof(*meshPointsPtr)); // Must be freed in Swift after method call
    for (size_t i = 0; i < meshNumPoints; i++)
    {
        meshPointsPtr[i].x = meshXYZPointCloud.points[i].x;
        meshPointsPtr[i].y = meshXYZPointCloud.points[i].y;
        meshPointsPtr[i].z = meshXYZPointCloud.points[i].z;
    }
    
    PCLPolygon *meshPolygonsPtr;
    meshPolygonsPtr = (PCLPolygon *) calloc(meshNumFaces, sizeof(*meshPolygonsPtr)); // Must be freed in Swift after method call
    for (size_t i = 0; i < meshNumFaces; i++)
    {
        // Are all faces always triangles?
        PCLPolygon pclPolygon;
        pclPolygon.v1 = mesh.polygons[i].vertices[0];
        pclPolygon.v2 = mesh.polygons[i].vertices[1];
        pclPolygon.v3 = mesh.polygons[i].vertices[2];
        meshPolygonsPtr[i] = pclPolygon;
    }
    
    PCLMesh pclMesh;
    pclMesh.numPoints = meshNumPoints;
    pclMesh.numFaces = meshNumFaces;
    pclMesh.points = meshPointsPtr;
    pclMesh.polygons = meshPolygonsPtr;
    return pclMesh;
}
