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
 Exports an SCNGeometry object to a user's Google Drive account as a file.
 */
internal class SurfaceExporter {
    
    private let googleDriveUploader = GoogleDriveUploader() // TODO: Remove GDrive dependency in this class
    
    /**
     Exports the given SCNGeometry object to a user's Google Drive account as a file with the given name and file extension.
     Takes a success callback function to be called if the export succeeds and a failure callback for if the export fails.
     */
    internal func exportSurface(fileNamed fileName: String,
                                withGeometry surfaceGeometry: SCNGeometry?,
                                withExtension fileExtension: String,
                                onSuccess successCallback: () -> (),
                                onFailure failureCallback: () -> (),
                                service: GTLRDriveService
                                ) {
        
        guard let surfaceGeometry = surfaceGeometry else {
            return
        }
        
        // Create MDLAsset that can be exported
        var mdlAsset = MDLAsset()
        let mdlMesh = MDLMesh(scnGeometry: surfaceGeometry)
        mdlAsset = MDLAsset(bufferAllocator: mdlMesh.allocator)
        mdlAsset.add(mdlMesh)
        
        do {
            try uploadAsset(mdlAsset: mdlAsset, withFileName: fileName, withExtension: fileExtension, withService: service)
            successCallback()
        } catch {
            failureCallback()
        }
    }
    
    /**
     Types of errors that could happen in the export process.
     */
    private enum ExportError: Error {
        case fileExportError
    }

    /**
     Creates a temporary file from an MDLAsset, uploads to google drive, and deletes the temporary file.
     */
    private func uploadAsset(mdlAsset: MDLAsset,
                            withFileName fileName: String,
                            withExtension fileExtension: String,
                            withService service: GTLRDriveService
        ) throws {
        
        // Create temporary file
        let fileManager = FileManager.default
        var tempFileURL = URL(fileURLWithPath: fileName, relativeTo: fileManager.temporaryDirectory)
        tempFileURL.appendPathExtension(fileExtension)
        
        do {
            // Export mesh to temporary file
            try mdlAsset.export(to: tempFileURL)
            
            // Read from file
            let surfaceFileContents = try Data(contentsOf: tempFileURL)
            
            // Upload file
            try googleDriveUploader.uploadDataFile(
                service: service,
                fileData: surfaceFileContents,
                name: fileName,
                fileExtension: fileExtension
            )
            
            // Discard file
            try fileManager.removeItem(at: tempFileURL)
        } catch {
            throw ExportError.fileExportError
        }
    }
}
