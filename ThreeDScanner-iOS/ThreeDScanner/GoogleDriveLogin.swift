//
//  GoogleDriveLogin.swift
//  ThreeDScanner
//
//  Created by Steven Roach on 4/4/18.
//  Copyright © 2018 Steven Roach. All rights reserved.
//

import Foundation
import GoogleAPIClientForREST
import GoogleSignIn

internal class GoogleDriveLogin {
    
    internal let service = GTLRDriveService()
    
    static let sharedInstance = GoogleDriveLogin()
    private init() {}
}
