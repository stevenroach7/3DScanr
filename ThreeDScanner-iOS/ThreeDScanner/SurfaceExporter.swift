//
//  SurfaceExporter.swift
//  ThreeDScanner
//
//  Created by Steven Roach on 4/3/18.
//  Copyright © 2018 Steven Roach. All rights reserved.
//

import Foundation
import SceneKit.ModelIO
import GoogleAPIClientForREST

/**
 Exports an SCNGeometry object to a Data file.
 */
internal class SurfaceExporter {
    
    /**
     Types of errors that could happen in the export process.
     */
    internal enum ExportError: Error {
        case fileExportError
    }
    
    /**
     Exports the given SCNGeometry object to a Data object and returns it.
     Creates a temporary file from the MDLAsset, reads from it, deletes, and returns the Data.
     */
    internal func exportSurface(withGeometry surfaceGeometry: SCNGeometry,
                                fileNamed fileName: String,
                                withExtension fileExtension: String
                                ) throws -> Data {
    
        // Create MDLAsset that can be exported
        var mdlAsset = MDLAsset()
        let mdlMesh = MDLMesh(scnGeometry: surfaceGeometry)
        mdlAsset = MDLAsset(bufferAllocator: mdlMesh.allocator)
        mdlAsset.add(mdlMesh)
        
        // Create temporary file
        let fileManager = FileManager.default
        var tempFileURL = URL(fileURLWithPath: fileName, relativeTo: fileManager.temporaryDirectory)
        tempFileURL.appendPathExtension(fileExtension)
        
        do {
            // Export mesh to temporary file
            try mdlAsset.export(to: tempFileURL)
            
            // Read from file
            let surfaceFileContents = try Data(contentsOf: tempFileURL)
            
            // Discard file
            try fileManager.removeItem(at: tempFileURL)
            
            return surfaceFileContents
        } catch {
            throw ExportError.fileExportError
        }
    }
}
