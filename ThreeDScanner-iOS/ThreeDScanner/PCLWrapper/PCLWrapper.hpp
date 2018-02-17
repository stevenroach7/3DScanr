//
//  PCLWrapper.hpp
//  ThreeDScanner
//
//  Created by Steven Roach on 2/9/18.
//  Copyright © 2018 Steven Roach. All rights reserved.
//

#ifndef PCLWrapper_hpp
#define PCLWrapper_hpp

#ifdef __cplusplus
extern "C" {
#endif
    
    // Input
    
    typedef struct PCLPoint3D {
        double x, y, z;
    } PCLPoint3D;
    
    typedef struct PCLNormal3D {
        double nx, ny, nz;
    } PCLNormal3D;
    
    typedef struct PCLPointCloud {
        int numPoints;
        PCLPoint3D *points;
//        PCLColor *colors;
    } PCLPointCloud;
    
    // Output
    
    typedef struct PCLPolygon {
        int v1, v2, v3;
    } PCLPolygon;
    
    typedef struct PCLMesh {
        long int numPoints;
        long int numFaces;
        PCLPoint3D *points;
        PCLNormal3D *normals;
        PCLPolygon *polygons;
    } PCLMesh;
    
    
    // Header declarations
    
    PCLMesh performSurfaceReconstruction(PCLPointCloud pointCloud);
    
#ifdef __cplusplus
}
#endif

#endif /* PCLWrapper_hpp */
