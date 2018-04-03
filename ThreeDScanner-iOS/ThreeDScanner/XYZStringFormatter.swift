//
//  TextFileFormatter.swift
//  ThreeDScanner
//
//  Created by Steven Roach on 4/2/18.
//  Copyright © 2018 Steven Roach. All rights reserved.
//

import Foundation
import SceneKit

/**
 Helper functions to format a collection of points into specified string formats.
 */
internal class XYZStringFormatter {
    
    /**
     Creates a string in XYZ file format from the given points.
     */
    internal func createXyzString(points: [float3]) -> String {
        var xyzString = "\n"
        for point in points {
            xyzString.append(point.x.description)
            xyzString.append(";")
            xyzString.append(point.y.description)
            xyzString.append(";")
            xyzString.append(point.z.description)
            xyzString.append("\n")
        }
        return xyzString
    }
    

    /**
     Creates a string in XYZRGB file format from the given points and colors.
     Assumes points and pointColors have the same length.
     */
    internal func createXyzRgbString(points: [float3], pointColors: [UIColor?]) -> String {
        
        var xyzRgbString = "\n"
        for i in 0..<points.count {
            let point = points[i]
            let color = pointColors[i]
            
            xyzRgbString.append(point.x.description)
            xyzRgbString.append(";")
            xyzRgbString.append(point.y.description)
            xyzRgbString.append(";")
            xyzRgbString.append(point.z.description)
            xyzRgbString.append(";")
            
            if let pointColor = color {
                let ciColor = CIColor(color: pointColor)
                xyzRgbString.append((ciColor.red * 255).description)
                xyzRgbString.append(";")
                xyzRgbString.append((ciColor.green * 255).description)
                xyzRgbString.append(";")
                xyzRgbString.append((ciColor.blue * 255).description)
            } else {
                xyzRgbString += ";;"
            }
            xyzRgbString += "\n"
        }
        return xyzRgbString
    }
}
