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
PointCloud<Normal>::Ptr computeNormals(PointCloud<PointXYZ>::Ptr pointCloud)
{
    cout << "Begin normal estimation" << endl;
    search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>());
    NormalEstimationOMP<PointXYZ, Normal> ne;
    ne.setSearchMethod(tree);
    ne.setNumberOfThreads(8);
    ne.setInputCloud(pointCloud); // FIXME: Is this bad?
    ne.setKSearch(100);
    // Compute the centroid of pointcloud
    Eigen::Vector4f centroid;
    compute3DCentroid(*pointCloud, centroid);
    ne.setViewPoint(centroid[0], centroid[1], centroid[2]);
    // Compute normals
    PointCloud<Normal>::Ptr cloudNormals(new PointCloud<Normal>());
    ne.compute(*cloudNormals);
    cout << "Normal estimation complete" << endl;
    
    // Reverse normals
    cout << "Reverse normals' direction" << endl;
    for (size_t i = 0; i < cloudNormals->size(); ++i) {
        cloudNormals->points[i].normal_x *= -1;
        cloudNormals->points[i].normal_y *= -1;
        cloudNormals->points[i].normal_z *= -1;
    }
    return cloudNormals;
}

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
    PointCloud<PointXYZ>::Ptr filteredPointCloud(new PointCloud<PointXYZ>());
    PassThrough<PointXYZ> filter;
    filter.setInputCloud(cloud);
    filter.filter(*filteredPointCloud);
    cout << "Passthrough filter complete" << endl;
    
    PointCloud<Normal>::Ptr pointCloudNormals = computeNormals(filteredPointCloud);
    
    cout << "Combine points and normals" << endl;
    PointCloud<PointNormal>::Ptr cloudSmoothedNormals(new PointCloud<PointNormal>());
    concatenateFields(*filteredPointCloud, *pointCloudNormals, *cloudSmoothedNormals);
    
    cout << "Begin poisson reconstruction" << endl;
    Poisson<PointNormal> poisson;
    poisson.setDepth(6);
    poisson.setInputCloud(cloudSmoothedNormals);
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
    
    
//    // TODO: Construct normals
//    PointCloud<Normal>::Ptr meshCloudNormals = computeNormals(meshXYZPointCloud.makeShared());
//
//    cout << "Mesh number of normals: " << meshCloudNormals->size() << endl;
//
//    PCLNormal3D *meshNormals;
//    meshNormals = (PCLNormal3D *) calloc(meshCloudNormals->size(), sizeof(*meshNormals));
//    for (size_t i = 0; i < meshCloudNormals->size(); i++)
//    {
//        PCLNormal3D pclNormal;
//        pclNormal.nx = meshCloudNormals->points[i].normal_x;
//        pclNormal.ny = meshCloudNormals->points[i].normal_y;
//        pclNormal.nz = meshCloudNormals->points[i].normal_z;
//        meshNormals[i] = pclNormal;
//    }
    
    PCLMesh pclMesh;
    pclMesh.numPoints = meshNumPoints;
    pclMesh.numFaces = meshNumFaces;
    pclMesh.points = meshPoints;
    pclMesh.polygons = meshPolygons;
//    pclMesh.normals = meshNormals;
    
    cout << "6" << endl;
    
    return pclMesh;
}
