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
PointCloud<Normal>::Ptr computeNormals(PointCloud<PointXYZ>::Ptr pointCloudPtr, PCLPoint3D viewpoint)
{
    search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>());
    NormalEstimationOMP<PointXYZ, Normal> ne;
    ne.setSearchMethod(tree);
    ne.setNumberOfThreads(8);
    ne.setInputCloud(pointCloudPtr);
    ne.setKSearch(100);
    ne.setViewPoint(viewpoint.x, viewpoint.y, viewpoint.z);
    
    // Compute normals
    PointCloud<Normal>::Ptr cloudNormalsPtr(new PointCloud<Normal>());
    ne.compute(*cloudNormalsPtr);
    return cloudNormalsPtr;
}



PointCloud<PointNormal>::Ptr constructPointNormalCloud(PCLPointCloud inputPCLPointCloud) {
     cout << "Constructing Point Cloud with normals" << endl;
    
    // Initalize Empty Point Cloud
    PointCloud<PointNormal>::Ptr pointCloudPtr(new PointCloud<PointNormal>);
    pointCloudPtr->width    = 0; // Always size of cloud
    pointCloudPtr->height   = 1; // Always 1
    pointCloudPtr->is_dense = false;
    pointCloudPtr->points.resize (pointCloudPtr->width * pointCloudPtr->height); // Need this line
    
    int currentPointsIdx = 0;
    for (size_t frameIdx = 0; frameIdx < inputPCLPointCloud.numFrames; frameIdx++) {
        
        int framePointCloudSize = inputPCLPointCloud.pointFrameLengths[frameIdx];
        
        PointCloud<PointXYZ>::Ptr tempPointCloudPtr(new PointCloud<PointXYZ>);
        tempPointCloudPtr->width    = framePointCloudSize; // Always size of cloud
        tempPointCloudPtr->height   = 1; // Always 1
        tempPointCloudPtr->is_dense = false;
        tempPointCloudPtr->points.resize (tempPointCloudPtr->width * tempPointCloudPtr->height);

        for (size_t i = 0; i < framePointCloudSize; i++, currentPointsIdx++)
        {
            tempPointCloudPtr->points[i].x = inputPCLPointCloud.points[currentPointsIdx].x;
            tempPointCloudPtr->points[i].y = inputPCLPointCloud.points[currentPointsIdx].y;
            tempPointCloudPtr->points[i].z = inputPCLPointCloud.points[currentPointsIdx].z;
        }
        PointCloud<Normal>::Ptr tempPointCloudNormalsPtr = computeNormals(tempPointCloudPtr, inputPCLPointCloud.viewpoints[frameIdx]);
        
        // Combine Points and Normals
        PointCloud<PointNormal>::Ptr tempCloudSmoothedNormalsPtr(new PointCloud<PointNormal>());
        concatenateFields(*tempPointCloudPtr, *tempPointCloudNormalsPtr, *tempCloudSmoothedNormalsPtr);
        
        // Append temp cloud to full cloud
        *pointCloudPtr += *tempCloudSmoothedNormalsPtr;
    }
    
    // Sanity Check
    cout << "Num points = " << inputPCLPointCloud.numPoints << ", Last Current Points Index = " << currentPointsIdx << endl;
    
    return pointCloudPtr;
}


PCLMesh performSurfaceReconstruction(PCLPointCloud inputPCLPointCloud) {
    
    PointCloud<PointNormal>::Ptr cloudSmoothedNormalsPtr = constructPointNormalCloud(inputPCLPointCloud);
    // Now filter if necessary
    
    cout << "Loaded Point Cloud with normals" << endl;

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
