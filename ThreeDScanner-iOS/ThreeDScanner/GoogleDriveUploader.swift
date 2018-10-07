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
     Uploads a text file the service google drive account from the given String, name, and extension.
     */
    internal func uploadTextFile(input: String, name: String, fileExtension: String = "txt", folderName: String? = nil) throws {
        let fileData = input.data(using: .utf8)!
        try uploadDataFile(fileData: fileData, name: name, fileExtension: fileExtension, folderName: folderName)
    }
    
    /**
     Uploads a file to the service google drive account with the given fileData, name, and extension.
     */
    internal func uploadDataFile(fileData: Data, name: String, fileExtension: String, folderName: String? = nil) throws {
        let metadata = GTLRDrive_File()
        metadata.name = name + ".\(fileExtension)"
        if let folderName = folderName {
            metadata.parents = [folderName]
        }
        
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
    
    internal func uploadFolder(name: String, closure: @escaping (_ folderID: String) -> Void) throws {
        print("Upload folder called!")
        let metadata = GTLRDrive_File()
        metadata.name = name
        metadata.mimeType = "application/vnd.google-apps.folder"

        let query = GTLRDriveQuery_FilesCreate.query(withObject: metadata, uploadParameters: nil)
        GoogleDriveLogin.sharedInstance.service.executeQuery(query, completionHandler: {(ticket:GTLRServiceTicket, object: Any?, error:Error?) in
            print("Completion handler called!®")
            if error == nil {
                let file = object as? GTLRDrive_File
                let folderID = (file?.identifier)!
                closure(folderID)
                print("Folder Upload Success")
            } else {
                print("An error occurred: \(String(describing: error))")
                throw UploadError.fileUploadError
            }
            } as? GTLRServiceCompletionHandler)
        print("upload folder done!")
    }
}
