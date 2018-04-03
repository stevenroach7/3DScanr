//
//  AppDelegate.swift
//  ThreeDScanner
//
//  Created by Steven Roach on 11/23/17.
//  Copyright © 2017 Steven Roach. All rights reserved.
//

import Google
import GoogleSignIn
import UIKit

@UIApplicationMain
class AppDelegate: UIResponder, UIApplicationDelegate {
    
    var window: UIWindow?
    
    func applicationDidFinishLaunching(_ application: UIApplication) {
        // Initialize sign-in
        var configureError: NSError?
        GGLContext.sharedInstance().configureWithError(&configureError)
        assert(configureError == nil, "Error configuring Google services: \(String(describing: configureError))")
    }
    
    func application(_ application: UIApplication,
                     open url: URL, sourceApplication: String?, annotation: Any) -> Bool {
        return GIDSignIn.sharedInstance().handle(url,
                                                 sourceApplication: sourceApplication,
                                                 annotation: annotation)
    }
    
    @available(iOS 9.0, *)
    func application(_ app: UIApplication, open url: URL,
                     options: [UIApplicationOpenURLOptionsKey : Any]) -> Bool {
        let sourceApplication = options[UIApplicationOpenURLOptionsKey.sourceApplication] as? String
        let annotation = options[UIApplicationOpenURLOptionsKey.annotation]
        return GIDSignIn.sharedInstance().handle(url,
                                                 sourceApplication: sourceApplication,
                                                 annotation: annotation)
    }
}

