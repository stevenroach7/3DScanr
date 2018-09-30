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
#include <pcl/filters/statistical_outlier_removal.h>
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
    ne.setKSearch(10);
    ne.setViewPoint(viewpoint.x, viewpoint.y, viewpoint.z);
    
    // Compute normals
    PointCloud<Normal>::Ptr cloudNormalsPtr(new PointCloud<Normal>());
    ne.compute(*cloudNormalsPtr);
    return cloudNormalsPtr;
}

PointCloud<PointXYZ>::Ptr filterPointCloudPerFrame(PointCloud<PointXYZ>::Ptr pointCloudPtr) {
    
    // Filtering Statistically
    StatisticalOutlierRemoval<PointXYZ> statFilter;
    statFilter.setInputCloud(pointCloudPtr);
    statFilter.setMeanK(((int) pointCloudPtr->size() - 1));
    statFilter.setStddevMulThresh(50 / pointCloudPtr->size()); // Should depend on size of cloud
    PointCloud<PointXYZ>::Ptr filteredPointCloudPtr(new PointCloud<PointXYZ>);
    statFilter.filter(*filteredPointCloudPtr);
    return filteredPointCloudPtr;
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
        
        PointCloud<PointXYZ>::Ptr tempFilteredPointCloudPtr = filterPointCloudPerFrame(tempPointCloudPtr);
        
        PointCloud<Normal>::Ptr tempPointCloudNormalsPtr = computeNormals(tempFilteredPointCloudPtr, inputPCLPointCloud.viewpoints[frameIdx]);
        
        // Combine Points and Normals
        PointCloud<PointNormal>::Ptr tempCloudSmoothedNormalsPtr(new PointCloud<PointNormal>());
        concatenateFields(*tempFilteredPointCloudPtr, *tempPointCloudNormalsPtr, *tempCloudSmoothedNormalsPtr);
        
        // Append temp cloud to full cloud
        *pointCloudPtr += *tempCloudSmoothedNormalsPtr;
    }
    
    // Sanity Check
    cout << "Num points = " << inputPCLPointCloud.numPoints << ", Last Current Points Index = " << currentPointsIdx << endl;
    
    return pointCloudPtr;
}

PCLPointNormalCloud constructPointCloudWithNormalsForTesting(PCLPointCloud inputPCLPointCloud) {
    
    PointCloud<PointNormal>::Ptr pointNormalCloud = constructPointNormalCloud(inputPCLPointCloud);
    
    // Convert to output format
    long int numPoints = pointNormalCloud->size();
    
    PCLPoint3D *pointsPtr;
    pointsPtr = (PCLPoint3D *) calloc(numPoints, sizeof(*pointsPtr)); // Must be freed in Swift after method call
    PCLPoint3D *normalsPtr;
    normalsPtr = (PCLPoint3D *) calloc(numPoints, sizeof(*normalsPtr)); // Must be freed in Swift after method call
    for (size_t i = 0; i < numPoints; i++)
    {
        pointsPtr[i].x = pointNormalCloud->points[i].x;
        pointsPtr[i].y = pointNormalCloud->points[i].y;
        pointsPtr[i].z = pointNormalCloud->points[i].z;
        normalsPtr[i].x = pointNormalCloud->points[i].normal_x;
        normalsPtr[i].y = pointNormalCloud->points[i].normal_y;
        normalsPtr[i].z = pointNormalCloud->points[i].normal_z;
    }

    PCLPointNormalCloud pclPointNormalCloud;
    pclPointNormalCloud.numPoints = (int) numPoints;
    pclPointNormalCloud.points = pointsPtr;
    pclPointNormalCloud.normals = normalsPtr;
    pclPointNormalCloud.numFrames = inputPCLPointCloud.numFrames;
    // These are pointing to memory managed by Swift but it shouldn't get cleaned up until after the Swift function finishes executing so is okay as a temporary measure
    pclPointNormalCloud.pointFrameLengths = inputPCLPointCloud.pointFrameLengths;
    pclPointNormalCloud.viewpoints = inputPCLPointCloud.viewpoints;
    
    return pclPointNormalCloud;
}

PCLMesh performSurfaceReconstruction(PCLPointCloud inputPCLPointCloud) {
    
    PointCloud<PointNormal>::Ptr pointNormalCloud = constructPointNormalCloud(inputPCLPointCloud);
    cout << "Loaded Point Cloud with normals" << endl;
    
    cout << "Statistically Filtering points" << endl;
    StatisticalOutlierRemoval<PointNormal> statFilter;
    statFilter.setInputCloud(pointNormalCloud);
    statFilter.setMeanK(50);
    statFilter.setStddevMulThresh(3); // 0.6 - 1.0
    
    PointCloud<PointNormal>::Ptr filteredPointCloudPtr(new PointCloud<PointNormal>);
    statFilter.filter(*filteredPointCloudPtr);
    cout << "Statistical points filtering complete" << endl;

    cout << "Begin poisson reconstruction" << endl;
    Poisson<PointNormal> poisson;
    poisson.setDepth(5); // Default is 6
    poisson.setInputCloud(filteredPointCloudPtr);
    poisson.setPointWeight(4);
    poisson.setSamplesPerNode(1.5);
    
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
