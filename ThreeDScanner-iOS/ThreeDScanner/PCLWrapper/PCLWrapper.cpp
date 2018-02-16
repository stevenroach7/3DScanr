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

PCLMesh performSurfaceReconstruction(PCLPointCloud pclPointCloud) {
    
    // Convert PCLPointCloud to PointCloud<XYZ>
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);

    cloud->width    = pclPointCloud.numPoints; // Always size of cloud
    cloud->height   = 1; // Always 1
    cloud->is_dense = false;
    cloud->points.resize (cloud->width * cloud->height); // Need this line
    
    cout << "Converting to PCL point cloud" << endl;
    cout << "Num points = " << pclPointCloud.numPoints << endl;

    for (size_t i = 0; i < pclPointCloud.numPoints; i++)
    {
        cloud->points[i].x = pclPointCloud.points[i].x;
        cloud->points[i].y = pclPointCloud.points[i].y;
        cloud->points[i].z = pclPointCloud.points[i].z;
    }

    cout << "Loaded Point Cloud" << endl;
    
    cout << "Begin passthrough filter" << endl;
    PointCloud<PointXYZ>::Ptr filtered(new PointCloud<PointXYZ>());
    PassThrough<PointXYZ> filter;
    filter.setInputCloud(cloud);
    filter.filter(*filtered);
    cout << "Passthrough filter complete" << endl;
    
    cout << "Begin normal estimation" << endl;
    search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>());
    NormalEstimationOMP<PointXYZ, Normal> ne;
    ne.setSearchMethod(tree);
    ne.setNumberOfThreads(8);
    ne.setInputCloud(filtered);
    ne.setKSearch(100);
    // Compute the centroid of pointcloud
    Eigen::Vector4f centroid;
    compute3DCentroid(*filtered, centroid);
    ne.setViewPoint(centroid[0], centroid[1], centroid[2]);
    // Compute normals
    PointCloud<Normal>::Ptr cloud_normals(new PointCloud<Normal>());
    ne.compute(*cloud_normals);
    cout << "Normal estimation complete" << endl;
    
    // Reverse normals
    cout << "Reverse normals' direction" << endl;
    for (size_t i = 0; i < cloud_normals->size(); ++i) {
        cloud_normals->points[i].normal_x *= -1;
        cloud_normals->points[i].normal_y *= -1;
        cloud_normals->points[i].normal_z *= -1;
    }
    
    cout << "Combine points and normals" << endl;
    PointCloud<PointNormal>::Ptr cloud_smoothed_normals(new PointCloud<PointNormal>());
    concatenateFields(*filtered, *cloud_normals, *cloud_smoothed_normals);
    
    cout << "Begin poisson reconstruction" << endl;
    Poisson<PointNormal> poisson;
    poisson.setDepth(9);
    poisson.setInputCloud(cloud_smoothed_normals);
    poisson.setPointWeight(0);
    poisson.setSamplesPerNode(1);
    
    PolygonMesh mesh;
    poisson.reconstruct(mesh);
    
    cout << "Mesh number of polygons: " << mesh.polygons.size() << endl;
    cout << "Poisson reconstruction complete" << endl;
    
    // Convert to output format
     cout << "1" << endl;
    
    
    // Need mesh cloud in PointCloud<PointXYZ> format instead of PointCloud2
    PointCloud<PointXYZ> meshXYZPointCloud;
    fromPCLPointCloud2(mesh.cloud, meshXYZPointCloud);
    
    cout << "2" << endl;
    
    long int meshNumPoints = meshXYZPointCloud.size();
    long int meshNumFaces = mesh.polygons.size();
    
    cout << "3" << endl;
    
    PCLPoint3D *meshPoints;
    meshPoints = (PCLPoint3D *) calloc(meshNumPoints, sizeof(*meshPoints));

    for (size_t i = 0; i < meshNumPoints; i++)
    {
        meshPoints[i].x = meshXYZPointCloud.points[i].x;
        meshPoints[i].y = meshXYZPointCloud.points[i].y;
        meshPoints[i].z = meshXYZPointCloud.points[i].z;
    }
    
    cout << "4" << endl;
    
    PCLPolygon *meshPolygons;
    meshPolygons = (PCLPolygon *) calloc(meshNumFaces, sizeof(*meshPolygons));
    for (size_t i = 0; i < meshNumFaces; i++)
    {
        // Are all faces always triangles?
        PCLPolygon pclPolygon;
        pclPolygon.v1 = mesh.polygons[i].vertices[0];
        pclPolygon.v2 = mesh.polygons[i].vertices[1];
        pclPolygon.v3 = mesh.polygons[i].vertices[2];
        meshPolygons[i] = pclPolygon;
    }
    
    cout << "5" << endl;
    
    PCLMesh pclMesh;
    pclMesh.numPoints = meshNumPoints;
    pclMesh.numFaces = meshNumFaces;
    pclMesh.points = meshPoints;
    pclMesh.polygons = meshPolygons;
    
    
    cout << "6" << endl;
    
    return pclMesh;
}

// Will call PCL code here.
int test(int i)
{
    return i;
}

const int  createTestCloud()
{
    PointCloud<PointXYZ> cloud;

    // Fill in the cloud data
    cloud.width    = 5;
    cloud.height   = 1;
    cloud.is_dense = false;
    cloud.points.resize (cloud.width * cloud.height);
    
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
        cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }
    
    return 0;
}





