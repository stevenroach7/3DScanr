//
//  GoogleDriveUploader.swift
//  ThreeDScanner
//
//  Created by Steven Roach on 4/2/18.
//  Copyright © 2018 Steven Roach. All rights reserved.
//

import Foundation
import GoogleAPIClientForREST

/**
 Class to upload files to Google Drive.
 */
internal class GoogleDriveUploader {
    
    /**
     Types of errors that could happen in the upload process.
     */
    internal enum UploadError: Error {
        case fileUploadError
    }
    
    /**
     Uploads a file to the servic google drive account with the given fileData, name and extension.
     */
    internal func uploadDataFile(fileData: Data, name: String, fileExtension: String) throws {
        let metadata = GTLRDrive_File()
        metadata.name = name + ".\(fileExtension)"
        
        let uploadParameters: GTLRUploadParameters = GTLRUploadParameters(data: fileData, mimeType: "text/plain")
        uploadParameters.shouldUploadWithSingleRequest = true
        
        let query = GTLRDriveQuery_FilesCreate.query(withObject: metadata, uploadParameters: uploadParameters)
        GoogleDriveLogin.sharedInstance.service.executeQuery(query, completionHandler: {(ticket:GTLRServiceTicket, object:Any?, error:Error?) in
            if error == nil {
                print("Text File Upload Success")
            } else {
                print("An error occurred: \(String(describing: error))")
                throw UploadError.fileUploadError
            }
        } as? GTLRServiceCompletionHandler)
    }
}
