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

int performSurfaceReconstruction() {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in fake cloud data
    cloud->width    = 5;
    cloud->height   = 1;
    cloud->is_dense = false;
    cloud->points.resize (cloud->width * cloud->height);
    
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }
    
    cout << "loaded" << endl;
    
    cout << "begin passthrough filter" << endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PassThrough<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);
    filter.filter(*filtered);
    cout << "passthrough filter complete" << endl;
    
    cout << "MLS complete" << endl;
    
    cout << "begin normal estimation" << endl;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setSearchMethod(tree);
    ne.setNumberOfThreads(8);
    ne.setInputCloud(filtered);
    ne.setKSearch(100);
    // compute the centroid of pointcloud
    Eigen::Vector4f centroid;
    compute3DCentroid(*filtered, centroid);
    //ne.setViewPoint(centroid[0], centroid[1], centroid[2]);
    // compute normals
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());
    ne.compute(*cloud_normals);
    cout << "normal estimation complete" << std::endl;
    
    // reverse normals
    cout << "reverse normals' direction" << std::endl;
    for (size_t i = 0; i < cloud_normals->size(); ++i) {
        cloud_normals->points[i].normal_x *= -1;
        cloud_normals->points[i].normal_y *= -1;
        cloud_normals->points[i].normal_z *= -1;
    }
    
    cout << "combine points and normals" << endl;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals(new pcl::PointCloud<pcl::PointNormal>());
    concatenateFields(*filtered, *cloud_normals, *cloud_smoothed_normals);
    
    cout << "begin poisson reconstruction" << endl;
    pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setDepth(9);
    poisson.setInputCloud(cloud_smoothed_normals);
    poisson.setPointWeight(0);
    poisson.setSamplesPerNode(1);
    
    pcl::PolygonMesh mesh;
    poisson.reconstruct(mesh);
    cout << "poisson reconstruction complete" << endl;
    
    return 0;
}

// Will call PCL code here.
char* test()
{
    char *mymessage;
    mymessage = (char*)malloc(15*sizeof(char));
    strcpy(mymessage, "Hello World");
    return mymessage;
}


const int  createTestCloud()
{
    pcl::PointCloud<pcl::PointXYZ> cloud;

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





