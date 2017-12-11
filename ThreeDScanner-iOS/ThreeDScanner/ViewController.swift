//
//  ViewController.swift
//  ThreeDScanner
//
//  Created by Steven Roach on 11/23/17.
//  Copyright © 2017 Steven Roach. All rights reserved.
//

import UIKit
import SceneKit
import ARKit
import GoogleAPIClientForREST
import GoogleSignIn

class ViewController: UIViewController, ARSCNViewDelegate, GIDSignInDelegate, GIDSignInUIDelegate {

    @IBOutlet var sceneView: ARSCNView!
    var points: [vector_float3] = []
    var pointsParentNode = SCNNode()
    
    let hidePointRatio = 2 // Hide 1 / hidePointRatio of the points
    
    // If modifying these scopes, delete your previously saved credentials by
    // resetting the iOS simulator or uninstall the app.
    private let scopes = ["https://www.googleapis.com/auth/drive"]
    private let service = GTLRDriveService()
    let signInButton = GIDSignInButton()
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        // Set the view's delegate
        sceneView.delegate = self
        
        // Configure Google Sign-in.
        GIDSignIn.sharedInstance().delegate = self
        GIDSignIn.sharedInstance().uiDelegate = self
        GIDSignIn.sharedInstance().scopes = scopes
        GIDSignIn.sharedInstance().signInSilently()
        
        // Add the sign-in button.
        view.addSubview(signInButton)
        
        addUploadButton()
//        addResetButton()
//        addClearScreenButton()
        
        sceneView.scene.rootNode.addChildNode(pointsParentNode)
    }
    
    internal func sign(_ signIn: GIDSignIn!, didSignInFor user: GIDGoogleUser!, withError error: Error!) {
        if let error = error {
            showAlert(title: "Authentication Error", message: error.localizedDescription)
            self.service.authorizer = nil
        } else {
            self.signInButton.isHidden = true
            self.service.authorizer = user.authentication.fetcherAuthorizer()
        }
    }
    
    // Helper for showing an alert
    private func showAlert(title : String, message: String) {
        let alert = UIAlertController(
            title: title,
            message: message,
            preferredStyle: UIAlertControllerStyle.alert
        )
        let ok = UIAlertAction(
            title: "OK",
            style: UIAlertActionStyle.default,
            handler: nil
        )
        alert.addAction(ok)
        present(alert, animated: true, completion: nil)
    }
    
    private func createXyString(points: [float3]) -> String {
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
    
    private func addUploadButton() {
        let uploadButton = UIButton()
        view.addSubview(uploadButton)
        uploadButton.translatesAutoresizingMaskIntoConstraints = false
        uploadButton.setTitle("Upload", for: .normal)
        uploadButton.setTitleColor(UIColor.red, for: .normal)
        uploadButton.backgroundColor = UIColor.white.withAlphaComponent(0.6)
        uploadButton.layer.cornerRadius = 4
        uploadButton.contentEdgeInsets = UIEdgeInsets(top: 5, left: 5, bottom: 5, right: 5)
        uploadButton.addTarget(self, action: #selector(uploadFile(sender:)) , for: .touchUpInside)
        
        // Contraints
        uploadButton.bottomAnchor.constraint(equalTo: view.bottomAnchor, constant: -8.0).isActive = true
        uploadButton.centerXAnchor.constraint(equalTo: view.centerXAnchor, constant: 0.0).isActive = true
        uploadButton.heightAnchor.constraint(equalToConstant: 50)
    }
    
    private func addResetButton() {
        let resetButton = UIButton()
        view.addSubview(resetButton)
        resetButton.translatesAutoresizingMaskIntoConstraints = false
        resetButton.setTitle("Reset points", for: .normal)
        resetButton.setTitleColor(UIColor.red, for: .normal)
        resetButton.backgroundColor = UIColor.white.withAlphaComponent(0.6)
        resetButton.layer.cornerRadius = 4
        resetButton.contentEdgeInsets = UIEdgeInsets(top: 5, left: 5, bottom: 5, right: 5)
        resetButton.addTarget(self, action: #selector(resetPointsButtonTapped(sender:)) , for: .touchUpInside)
        
        // Contraints
        resetButton.bottomAnchor.constraint(equalTo: view.bottomAnchor, constant: -8.0).isActive = true
        resetButton.rightAnchor.constraint(equalTo: view.rightAnchor, constant: -8.0).isActive = true
        resetButton.heightAnchor.constraint(equalToConstant: 50)
    }
    
    private func addClearScreenButton() {
        let clearScreenButton = UIButton()
        view.addSubview(clearScreenButton)
        clearScreenButton.translatesAutoresizingMaskIntoConstraints = false
        clearScreenButton.setTitle("Hide Half", for: .normal)
        clearScreenButton.setTitleColor(UIColor.red, for: .normal)
        clearScreenButton.backgroundColor = UIColor.white.withAlphaComponent(0.6)
        clearScreenButton.layer.cornerRadius = 4
        clearScreenButton.contentEdgeInsets = UIEdgeInsets(top: 5, left: 5, bottom: 5, right: 5)
        clearScreenButton.addTarget(self, action: #selector(clearScreenButtonTapped(sender:)) , for: .touchUpInside)
        
        // Contraints
        clearScreenButton.bottomAnchor.constraint(equalTo: view.bottomAnchor, constant: -8.0).isActive = true
        clearScreenButton.leftAnchor.constraint(equalTo: view.leftAnchor, constant: 8.0).isActive = true
        clearScreenButton.heightAnchor.constraint(equalToConstant: 50)
    }
    
    @IBAction func uploadFile(sender: UIButton) {
        
        let input = createXyString(points: points)
        let fileData = input.data(using: .utf8)!
        
        let dateFormatter = DateFormatter()
        dateFormatter.dateStyle = DateFormatter.Style.short
        dateFormatter.timeStyle = DateFormatter.Style.short
        let timeDateString = dateFormatter.string(from: Date())
        
        let metadata = GTLRDrive_File()
        metadata.name = "Points" + timeDateString
        
        let uploadParameters: GTLRUploadParameters = GTLRUploadParameters(data: fileData, mimeType: "text/plain")
        uploadParameters.shouldUploadWithSingleRequest = true
        
        let query = GTLRDriveQuery_FilesCreate.query(withObject: metadata, uploadParameters: uploadParameters)
        self.service.executeQuery(query, completionHandler: {(ticket:GTLRServiceTicket, object:Any?, error:Error?) in
            if error == nil {
                print("Succeed")
            }
            else {
                print("An error occurred: \(String(describing: error))")
            }
        })
    }
    
    @IBAction func resetPointsButtonTapped(sender: UIButton) {
        points = []
        pointsParentNode.enumerateChildNodes { (node, stop) -> Void in
            node.removeFromParentNode()
        }
    }
    
    @IBAction func clearScreenButtonTapped(sender: UIButton) {
        var i = 0
        pointsParentNode.enumerateChildNodes { (node, stop) -> Void in
            if (i % hidePointRatio != 0) {
                node.removeFromParentNode()
            }
            i+=1
        }
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        
        // Create a session configuration
        let configuration = ARWorldTrackingConfiguration()
        configuration.planeDetection = ARWorldTrackingConfiguration.PlaneDetection.horizontal

        // Run the view's session
        sceneView.session.run(configuration)
        
        // Show feature points
        sceneView.debugOptions.insert(ARSCNDebugOptions.showFeaturePoints)
        sceneView.debugOptions.insert(ARSCNDebugOptions.showWorldOrigin)
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        
        // Pause the view's session
        sceneView.session.pause()
    }
    
    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Release any cached data, images, etc that aren't in use.
        print("Memory Warning")
    }

    // MARK: - ARSCNViewDelegate
    
    func session(_ session: ARSession, didFailWithError error: Error) {
        // Present an error message to the user
        
    }
    
    func sessionWasInterrupted(_ session: ARSession) {
        // Inform the user that the session has been interrupted, for example, by presenting an overlay
        
    }
    
    func sessionInterruptionEnded(_ session: ARSession) {
        // Reset tracking and/or remove existing anchors if consistent tracking is required
        
    }
    
    override func touchesBegan(_ touches: Set<UITouch>, with event: UIEvent?) {
        guard let rawFeaturePoints = sceneView.session.currentFrame?.rawFeaturePoints else {
            return
        }
        for rawPoint in rawFeaturePoints.points {
            addPointToView(position: rawPoint)
        }
        points += rawFeaturePoints.points
    }
    
    private func addPointToView(position: vector_float3) {
        let sphere = SCNSphere(radius: 0.00066)
        let sphereNode = SCNNode(geometry: sphere)
        sphereNode.position = SCNVector3(position)
        pointsParentNode.addChildNode(sphereNode)
    }
}
