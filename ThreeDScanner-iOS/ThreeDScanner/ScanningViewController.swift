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
import SceneKit.ModelIO

class ScanningViewController: UIViewController, ARSCNViewDelegate, SCNSceneRendererDelegate, GIDSignInDelegate, GIDSignInUIDelegate {

    // MARK: - Properties
    
    @IBOutlet var sceneView: ARSCNView!
    private let sessionConfiguration = ARWorldTrackingConfiguration()
    private var points: [vector_float3] = []
    private var pointCloudFrameSizes: [Int32] = []
    private var pointCloudFrameViewpoints: [SCNVector3] = []
    
    private let xyzStringFormatter = XYZStringFormatter()
    
    private var pointsParentNode = SCNNode()
    private var surfaceParentNode = SCNNode()
    
    private var isTorchOn = false
    private var addPointRatio = 3 // Show 1 / addPointRatio of the points
    
    private let exportExtensionString = "stl"
    private let imageQuality = 0.85 // Value between 0 and 1
    private var pointMaterial: SCNMaterial?
    private var surfaceGeometry: SCNGeometry?

    // If modifying these scopes, delete your previously saved credentials by
    // resetting the iOS simulator or uninstall the app.
    private let scopes = ["https://www.googleapis.com/auth/drive"]
    private let service = GTLRDriveService()
    private let signInButton = GIDSignInButton()

    
    // MARK: - UIViewController
    
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
        
        addReconstructButton()
        addToggleTorchButton()
        addResetButton()
        addOptionsButton()
        addExportButton()
        
        createPointMaterial()
        
        sceneView.scene.rootNode.addChildNode(pointsParentNode)
        sceneView.scene.rootNode.addChildNode(surfaceParentNode)
        
        sceneView.autoenablesDefaultLighting = true
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        
        // Set Session configuration
        sessionConfiguration.planeDetection = ARWorldTrackingConfiguration.PlaneDetection.horizontal
        
        // Run the view's session
        sceneView.session.run(sessionConfiguration)
        
        // Show feature points and world origin
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
    
    
    // MARK: - UI
    
    private func addReconstructButton() {
        let reconstructButton = UIButton()
        view.addSubview(reconstructButton)
        reconstructButton.translatesAutoresizingMaskIntoConstraints = false
        reconstructButton.setTitle("Reconstruct", for: .normal)
        reconstructButton.setTitleColor(UIColor.red, for: .normal)
        reconstructButton.backgroundColor = UIColor.white.withAlphaComponent(0.6)
        reconstructButton.layer.cornerRadius = 4
        reconstructButton.contentEdgeInsets = UIEdgeInsets(top: 5, left: 5, bottom: 5, right: 5)
        reconstructButton.addTarget(self, action: #selector(reconstructButtonTapped(sender:)) , for: .touchUpInside)
        
        // Contraints
        reconstructButton.bottomAnchor.constraint(equalTo: view.bottomAnchor, constant: -8.0).isActive = true
        reconstructButton.centerXAnchor.constraint(equalTo: view.centerXAnchor, constant: 0.0).isActive = true
        reconstructButton.heightAnchor.constraint(equalToConstant: 50)
    }
    
    private func addToggleTorchButton() {
        let toggleTorchButton = UIButton()
        view.addSubview(toggleTorchButton)
        toggleTorchButton.translatesAutoresizingMaskIntoConstraints = false
        toggleTorchButton.setTitle("Torch", for: .normal)
        toggleTorchButton.setTitleColor(UIColor.red, for: .normal)
        toggleTorchButton.backgroundColor = UIColor.white.withAlphaComponent(0.6)
        toggleTorchButton.layer.cornerRadius = 4
        toggleTorchButton.contentEdgeInsets = UIEdgeInsets(top: 5, left: 5, bottom: 5, right: 5)
        toggleTorchButton.addTarget(self, action: #selector(toggleButtonTapped(sender:)) , for: .touchUpInside)
        
        // Contraints
        toggleTorchButton.bottomAnchor.constraint(equalTo: view.bottomAnchor, constant: -8.0).isActive = true
        toggleTorchButton.rightAnchor.constraint(equalTo: view.rightAnchor, constant: -8.0).isActive = true
        toggleTorchButton.heightAnchor.constraint(equalToConstant: 50)
    }
    
    private func addResetButton() {
        let resetButton = UIButton()
        view.addSubview(resetButton)
        resetButton.translatesAutoresizingMaskIntoConstraints = false
        resetButton.setTitle("Reset", for: .normal)
        resetButton.setTitleColor(UIColor.red, for: .normal)
        resetButton.backgroundColor = UIColor.white.withAlphaComponent(0.6)
        resetButton.layer.cornerRadius = 4
        resetButton.contentEdgeInsets = UIEdgeInsets(top: 5, left: 5, bottom: 5, right: 5)
        resetButton.addTarget(self, action: #selector(resetButtonTapped(sender:)) , for: .touchUpInside)
        
        // Contraints
        resetButton.topAnchor.constraint(equalTo: view.topAnchor, constant: 20.0).isActive = true
        resetButton.rightAnchor.constraint(equalTo: view.rightAnchor, constant: -90.0).isActive = true
        resetButton.heightAnchor.constraint(equalToConstant: 50)
    }
    
    private func addOptionsButton() {
        let optionsButton = UIButton()
        view.addSubview(optionsButton)
        optionsButton.translatesAutoresizingMaskIntoConstraints = false
        optionsButton.setTitle("Options", for: .normal)
        optionsButton.setTitleColor(UIColor.red, for: .normal)
        optionsButton.backgroundColor = UIColor.white.withAlphaComponent(0.6)
        optionsButton.layer.cornerRadius = 4
        optionsButton.contentEdgeInsets = UIEdgeInsets(top: 5, left: 5, bottom: 5, right: 5)
        optionsButton.addTarget(self, action: #selector(optionsButtonTapped(sender:)) , for: .touchUpInside)
        
        // Contraints
        optionsButton.bottomAnchor.constraint(equalTo: view.bottomAnchor, constant: -8.0).isActive = true
        optionsButton.leftAnchor.constraint(equalTo: view.leftAnchor, constant: 8.0).isActive = true
        optionsButton.heightAnchor.constraint(equalToConstant: 50)
    }
    
    private func addExportButton() {
        let exportButton = UIButton()
        view.addSubview(exportButton)
        exportButton.translatesAutoresizingMaskIntoConstraints = false
        exportButton.setTitle("Export", for: .normal)
        exportButton.setTitleColor(UIColor.red, for: .normal)
        exportButton.backgroundColor = UIColor.white.withAlphaComponent(0.6)
        exportButton.layer.cornerRadius = 4
        exportButton.contentEdgeInsets = UIEdgeInsets(top: 5, left: 5, bottom: 5, right: 5)
        exportButton.addTarget(self, action: #selector(exportButtonTapped(sender:)) , for: .touchUpInside)
        
        // Contraints
        exportButton.topAnchor.constraint(equalTo: view.topAnchor, constant: 20.0).isActive = true
        exportButton.leftAnchor.constraint(equalTo: view.leftAnchor, constant: 90.0).isActive = true
        exportButton.heightAnchor.constraint(equalToConstant: 50)
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
    
    
    // MARK: - UI Actions
    
    @IBAction func reconstructButtonTapped(sender: UIButton) {
        
        let pclPoints = points.map { PCLPoint3D(x: Double($0.x), y: Double($0.y), z: Double($0.z)) }
        let pclViewpoints = pointCloudFrameViewpoints.map { PCLPoint3D(x: Double($0.x), y: Double($0.y), z: Double($0.z)) }
        
        let pclPointCloud = PCLPointCloud(numPoints: Int32(points.count),
                                          points: pclPoints,
                                          numFrames: Int32(pointCloudFrameViewpoints.count),
                                          pointFrameLengths: pointCloudFrameSizes,
                                          viewpoints: pclViewpoints)
        
        let pclMesh = performSurfaceReconstruction(pclPointCloud)
        defer {
            // The mesh points and polygons pointers were allocated in C so need to be freed here
            free(pclMesh.points)
            free(pclMesh.polygons)
        }
        
        print("mesh num points \(pclMesh.numPoints)")
        print("mesh num faces \(pclMesh.numFaces)")
        
        // Remove current surfaces before displaying new surface
        surfaceParentNode.enumerateChildNodes { (node, stop) in
            node.removeFromParentNode()
            node.geometry = nil
        }
        
        let surfaceNode = constructSurfaceNode(pclMesh: pclMesh)
        surfaceParentNode.addChildNode(surfaceNode)
        
        showAlert(title: "Surface Reconstructed", message: "\(pclMesh.numFaces) faces")
    }
    
    @IBAction func exportButtonTapped(sender: UIButton) {
        
        guard let surfaceGeometry = surfaceGeometry else {
            showAlert(title: "No Surface to Export", message: "Press Reconstruct Surface and then export.")
            return
        }
        
        // Create MDLAsset that can be exported
        var mdlAsset = MDLAsset()
        let mdlMesh = MDLMesh(scnGeometry: surfaceGeometry)
        mdlAsset = MDLAsset(bufferAllocator: mdlMesh.allocator)
        mdlAsset.add(mdlMesh)
        
        // Create temporary file
        let fileManager = FileManager.default
        
        var tempFileURL = URL(fileURLWithPath: "SurfaceModel", relativeTo: fileManager.temporaryDirectory)
        tempFileURL.appendPathExtension(exportExtensionString)
        do {
            // Export mesh to temporary file
            try mdlAsset.export(to: tempFileURL)

            // Read from file
            let surfaceFileContents = try Data(contentsOf: tempFileURL)
            
            // Upload file
            do {
                try uploadDataFile(fileData: surfaceFileContents, name: "SurfaceModel", fileExtension: exportExtensionString)
            } catch {
                showAlert(title: "Upload Error", message: "Please try again.")
                return
            }
            showAlert(title: "Sucessful Upload", message: "")

            // Discard file
            try fileManager.removeItem(at: tempFileURL)
        } catch {
            showAlert(title: "Error exporting to file", message: "")
            return
        }
    }
    
    @IBAction func resetButtonTapped(sender: UIButton) {
        
        points = []
        pointCloudFrameSizes = []
        pointCloudFrameViewpoints = []

        sceneView.scene.rootNode.enumerateChildNodes { (node, stop) in node.removeFromParentNode() }

        pointsParentNode = SCNNode()
        surfaceParentNode = SCNNode()

        sceneView.scene.rootNode.addChildNode(pointsParentNode)
        sceneView.scene.rootNode.addChildNode(surfaceParentNode)

        sceneView.debugOptions.remove(ARSCNDebugOptions.showFeaturePoints)
        sceneView.debugOptions.remove(ARSCNDebugOptions.showWorldOrigin)
        
        // Run the view's session
        sceneView.session.run(sessionConfiguration, options: [ARSession.RunOptions.resetTracking, ARSession.RunOptions.removeExistingAnchors])
        
        sceneView.debugOptions.insert(ARSCNDebugOptions.showFeaturePoints)
        sceneView.debugOptions.insert(ARSCNDebugOptions.showWorldOrigin)
    }
    
    @IBAction func toggleButtonTapped(sender: UIButton) {
        guard let device = AVCaptureDevice.default(for: AVMediaType.video) else {
            return
        }
        
        if device.hasTorch {
            do {
                try device.lockForConfiguration()
                
                if !isTorchOn {
                    device.torchMode = .on
                    try device.setTorchModeOn(level: AVCaptureDevice.maxAvailableTorchLevel)
                } else {
                    device.torchMode = .off
                }
                isTorchOn = !isTorchOn
                
                device.unlockForConfiguration()
            } catch {
                print("Torch could not be used")
            }
        } else {
            print("Torch is not available")
        }
    }
    
    @IBAction func optionsButtonTapped(sender: UIButton) {
        let alert = UIAlertController(title: "Adjust Add Point Ratio", message: "1 out of every _ points will be shown.", preferredStyle: UIAlertControllerStyle.alert)
        alert.addTextField(configurationHandler: {(textField: UITextField!) in
            textField.text = self.addPointRatio.description
            textField.keyboardType = UIKeyboardType.numberPad
        })
        
        alert.addAction(UIAlertAction(title: "Enter", style: UIAlertActionStyle.default, handler: { [weak alert] (_) in
            let textField = alert?.textFields![0] // Force unwrapping because we know it exists.
            if let text = textField!.text {
                if let newRatio = Int(text) {
                    self.addPointRatio = newRatio
                }
            }
        }))
        self.present(alert, animated: true, completion: nil)
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
    
    override func touchesBegan(_ touches: Set<UITouch>, with event: UIEvent?) {
        
        // Store Points
        guard let rawFeaturePoints = sceneView.session.currentFrame?.rawFeaturePoints else {
            return
        }
        let currentPoints = rawFeaturePoints.points
        points += currentPoints
        pointCloudFrameSizes.append(Int32(currentPoints.count))
        
        // Display points
        var i = 0
        for rawPoint in currentPoints {
            if i % addPointRatio == 0 {
                addPointToView(position: rawPoint)
            }
            i += 1
        }

        // Add viewpoint
        let camera = sceneView.session.currentFrame?.camera
        if let transform = camera?.transform {
            let position = SCNVector3(
                transform.columns.3.x,
                transform.columns.3.y,
                transform.columns.3.z
            )
            pointCloudFrameViewpoints.append(position)
        }
    }
    
    
    // MARK: - Surface Helper Functions
    
    private func constructSurfaceNode(pclMesh: PCLMesh) -> SCNNode {
        
        var vertices = [SCNVector3]()
        for i in 0..<pclMesh.numPoints {
            vertices.append(SCNVector3(x: Float(pclMesh.points[i].x),
                                       y: Float(pclMesh.points[i].y),
                                       z: Float(pclMesh.points[i].z)))
        }
        let vertexSource = SCNGeometrySource(vertices: vertices)
        
        var elements = [SCNGeometryElement]()
        for i in 0..<pclMesh.numFaces {
            let allPrimitives: [Int32] = [pclMesh.polygons[i].v1, pclMesh.polygons[i].v2, pclMesh.polygons[i].v3]
            elements.append(SCNGeometryElement(indices: allPrimitives, primitiveType: .triangles))
        }
        
        surfaceGeometry = SCNGeometry(sources: [vertexSource], elements: elements)
        if let surfaceGeometry = surfaceGeometry {
            surfaceGeometry.firstMaterial?.isDoubleSided = true;
            surfaceGeometry.firstMaterial?.diffuse.contents =
                UIColor(displayP3Red: 135/255, green: 206/255, blue: 250/255, alpha: 1)
            surfaceGeometry.firstMaterial?.lightingModel = .blinn
            return SCNNode(geometry: surfaceGeometry)
        }
        else { return SCNNode() } // FIXME: Maybe this should throw?
    }
    
    
    // MARK: - Google Drive Helper Functions
    
    enum UploadError: Error {
        case fileUploadError
    }
    
    private func uploadDataFile(fileData: Data, name: String, fileExtension: String) throws {
        let metadata = GTLRDrive_File()
        metadata.name = name + ".\(fileExtension)"
        
        let uploadParameters: GTLRUploadParameters = GTLRUploadParameters(data: fileData, mimeType: "text/plain")
        uploadParameters.shouldUploadWithSingleRequest = true
        
        let query = GTLRDriveQuery_FilesCreate.query(withObject: metadata, uploadParameters: uploadParameters)
        self.service.executeQuery(query, completionHandler: {(ticket:GTLRServiceTicket, object:Any?, error:Error?) in
            if error == nil {
                print("Text File Upload Success")
            } else {
                print("An error occurred: \(String(describing: error))")
                throw UploadError.fileUploadError
            }
            } as? GTLRServiceCompletionHandler)
    }
    
    
    // MARK: - Display Helper Functions
    
    private func createPointMaterial() {
        let textureImage = #imageLiteral(resourceName: "WhiteBlack")
        UIGraphicsBeginImageContext(textureImage.size)
        let width = textureImage.size.width
        let height = textureImage.size.height
        textureImage.draw(in: CGRect(x: 0, y: 0, width: width, height: height))
        let pointMaterialImage = UIGraphicsGetImageFromCurrentImageContext()
        UIGraphicsEndImageContext()
        pointMaterial = SCNMaterial()
        pointMaterial?.diffuse.contents = pointMaterialImage
    }
    
    private func addPointToView(position: vector_float3) {
        let sphere = SCNSphere(radius: 0.00066)
        sphere.segmentCount = 8
        
        if let pointMaterial = pointMaterial {
            sphere.firstMaterial = pointMaterial
        }
        let sphereNode = SCNNode(geometry: sphere)
        sphereNode.orientation = (sceneView.pointOfView?.orientation)!
        sphereNode.pivot = SCNMatrix4MakeRotation(-Float.pi / 2, 0, 1, 0)
        sphereNode.position = SCNVector3(position)
        pointsParentNode.addChildNode(sphereNode)
    }
}
