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
    
    typedef struct PCLPoint3D {
        double x, y, z;
    } PCLPoint3D;
    
    typedef struct PCLPointCloud {
        int numPoints;
        PCLPoint3D *points;
//        PCLColor *colors;
    } PCLPointCloud;

    int performSurfaceReconstruction(PCLPointCloud pointCloud);
    int test(int i);
    const int createTestCloud();
    
#ifdef __cplusplus
}
#endif

#endif /* PCLWrapper_hpp */
