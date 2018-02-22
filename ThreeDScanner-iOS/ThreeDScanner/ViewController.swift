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

class ViewController: UIViewController, ARSCNViewDelegate, SCNSceneRendererDelegate, GIDSignInDelegate, GIDSignInUIDelegate {

    // MARK: - Properties
    
    @IBOutlet var sceneView: ARSCNView!
    let sessionConfiguration = ARWorldTrackingConfiguration()
    var points: [vector_float3] = []
    var colors: [UIColor?] = []
    var pointCloudFrameSizes: [Int32] = []
    var pointCloudFrameViewpoints: [SCNVector3] = []
    var pointsParentNode = SCNNode()
    var surfaceParentNode = SCNNode()
    var isTorchOn = false
    var addPointRatio = 1 // Show 1 / addPointRatio of the points
    var folderID = ""
    var hasFolderBeenUploaded = false
    var isMultipartUploadOn = false {
        didSet {
            if isMultipartUploadOn {
                uploadFolder()
            }
        }
    }
    let pendingImageUploadLabel: UILabel = UILabel(frame: CGRect(x: 0, y: 0, width: 200, height: 21))
    var pendingImageUploads = 0 {
        didSet {
            pendingImageUploadLabel.text = "\(pendingImageUploads) pending uploads"
        }
    }
    let imageQuality = 0.85 // Value between 0 and 1
    var pointMaterial: SCNMaterial?
    var surfaceGeometry: SCNGeometry?

    // If modifying these scopes, delete your previously saved credentials by
    // resetting the iOS simulator or uninstall the app.
    private let scopes = ["https://www.googleapis.com/auth/drive"]
    private let service = GTLRDriveService()
    let signInButton = GIDSignInButton()
    
    
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
        addInfoButton()
        addResetButton()
        addOptionsButton()
        addMultipartUploadSwitch()
//        addPendingImageUploadLabel()
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
        reconstructButton.addTarget(self, action: #selector(reconstructSurface(sender:)) , for: .touchUpInside)
        
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
        toggleTorchButton.addTarget(self, action: #selector(toggleTorch(sender:)) , for: .touchUpInside)
        
        // Contraints
        toggleTorchButton.bottomAnchor.constraint(equalTo: view.bottomAnchor, constant: -8.0).isActive = true
        toggleTorchButton.rightAnchor.constraint(equalTo: view.rightAnchor, constant: -8.0).isActive = true
        toggleTorchButton.heightAnchor.constraint(equalToConstant: 50)
    }
    
    private func addInfoButton() {
        let infoButton = UIButton()
        view.addSubview(infoButton)
        infoButton.translatesAutoresizingMaskIntoConstraints = false
        infoButton.setTitle("Info", for: .normal)
        infoButton.setTitleColor(UIColor.red, for: .normal)
        infoButton.backgroundColor = UIColor.white.withAlphaComponent(0.6)
        infoButton.layer.cornerRadius = 4
        infoButton.contentEdgeInsets = UIEdgeInsets(top: 5, left: 5, bottom: 5, right: 5)
        infoButton.addTarget(self, action: #selector(showInfoPopup(sender:)) , for: .touchUpInside)
        
        // Contraints
        infoButton.topAnchor.constraint(equalTo: view.topAnchor, constant: 20.0).isActive = true
        infoButton.rightAnchor.constraint(equalTo: view.rightAnchor, constant: -8.0).isActive = true
        infoButton.heightAnchor.constraint(equalToConstant: 50)
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
        resetButton.addTarget(self, action: #selector(resetScene(sender:)) , for: .touchUpInside)
        
        // Contraints
        resetButton.topAnchor.constraint(equalTo: view.topAnchor, constant: 20.0).isActive = true
        resetButton.rightAnchor.constraint(equalTo: view.rightAnchor, constant: -55.0).isActive = true
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
        optionsButton.addTarget(self, action: #selector(showOptionsPopup(sender:)) , for: .touchUpInside)
        
        // Contraints
        optionsButton.bottomAnchor.constraint(equalTo: view.bottomAnchor, constant: -8.0).isActive = true
        optionsButton.leftAnchor.constraint(equalTo: view.leftAnchor, constant: 8.0).isActive = true
        optionsButton.heightAnchor.constraint(equalToConstant: 50)
    }
    
    private func addMultipartUploadSwitch() {
        let multipartUploadSwitch = UISwitch()
        view.addSubview(multipartUploadSwitch)
        multipartUploadSwitch.translatesAutoresizingMaskIntoConstraints = false
        multipartUploadSwitch.isOn = isMultipartUploadOn
        multipartUploadSwitch.setOn(isMultipartUploadOn, animated: false)
        multipartUploadSwitch.addTarget(self, action: #selector(switchValueDidChange(sender:)), for: .valueChanged)
        
        // Contraints
        multipartUploadSwitch.topAnchor.constraint(equalTo: view.topAnchor, constant: 20.0).isActive = true
        multipartUploadSwitch.leftAnchor.constraint(equalTo: view.leftAnchor, constant: 8.0).isActive = true
        multipartUploadSwitch.heightAnchor.constraint(equalToConstant: 50)
    }
    
    private func addPendingImageUploadLabel() {
        view.addSubview(pendingImageUploadLabel)
        pendingImageUploadLabel.translatesAutoresizingMaskIntoConstraints = false
        pendingImageUploadLabel.textAlignment = .center
        pendingImageUploadLabel.text = "\(pendingImageUploads) pending uploads"

        // Contraints
        pendingImageUploadLabel.topAnchor.constraint(equalTo: view.topAnchor, constant: 24.0).isActive = true
        pendingImageUploadLabel.centerXAnchor.constraint(equalTo: view.centerXAnchor, constant: 0.0).isActive = true
        pendingImageUploadLabel.heightAnchor.constraint(equalToConstant: 50)
    }
    
    private func addExportButton() {
        let exportButton = UIButton()
        view.addSubview(exportButton)
        exportButton.translatesAutoresizingMaskIntoConstraints = false
        exportButton.setTitle("Export Surface", for: .normal)
        exportButton.setTitleColor(UIColor.red, for: .normal)
        exportButton.backgroundColor = UIColor.white.withAlphaComponent(0.6)
        exportButton.layer.cornerRadius = 4
        exportButton.contentEdgeInsets = UIEdgeInsets(top: 5, left: 5, bottom: 5, right: 5)
        exportButton.addTarget(self, action: #selector(exportSurface(sender:)) , for: .touchUpInside)
        
        // Contraints
        exportButton.topAnchor.constraint(equalTo: view.topAnchor, constant: 20.0).isActive = true
        exportButton.centerXAnchor.constraint(equalTo: view.centerXAnchor, constant: 0.0).isActive = true
        exportButton.heightAnchor.constraint(equalToConstant: 50)
    }
    
    
    // MARK: - UI Actions
    
    @IBAction func reconstructSurface(sender: UIButton) {
        
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
        
        if isMultipartUploadOn {
            // For Testing
            let pclPointNormalTestingCloud = constructPointCloudWithNormalsForTesting(pclPointCloud)
            uploadPointNormalViewpointTextFiles(pclPointNormalCloud: pclPointNormalTestingCloud)
            defer {
                // The points and normals pointers were allocated in C so need to be freed here
                free(pclPointNormalTestingCloud.points)
                free(pclPointNormalTestingCloud.normals)
            }
        }
        
        print("mesh num points \(pclMesh.numPoints)")
        print("mesh num faces \(pclMesh.numFaces)")
        
        // Remove current surfaces before displaying new surface
        surfaceParentNode.enumerateChildNodes { (node, stop) in node.removeFromParentNode() }
        
        let surfaceNode = constructSurfaceNode(pclMesh: pclMesh)
        surfaceParentNode.addChildNode(surfaceNode)
        
        showAlert(title: "Surface Reconstructed", message: "\(pclMesh.numFaces) faces")
    }
    
    private func uploadPointNormalViewpointTextFiles(pclPointNormalCloud: PCLPointNormalCloud) {
        
        var currentPointsIdx = 0
        
        var allPointsNormalsString = "\n"
        
        for frameIdx in 0..<Int(pclPointNormalCloud.numFrames) {
            
            var framePointsNormalsString = ""
            framePointsNormalsString.append("\(pclPointNormalCloud.viewpoints[frameIdx].x);")
            framePointsNormalsString.append("\(pclPointNormalCloud.viewpoints[frameIdx].y);")
            framePointsNormalsString.append("\(pclPointNormalCloud.viewpoints[frameIdx].z)\n")
            
            for _ in 0..<pclPointNormalCloud.pointFrameLengths[frameIdx] {
                
                var pointNormalLineString = ""
                pointNormalLineString.append("\(pclPointNormalCloud.points[currentPointsIdx].x);")
                pointNormalLineString.append("\(pclPointNormalCloud.points[currentPointsIdx].y);")
                pointNormalLineString.append("\(pclPointNormalCloud.points[currentPointsIdx].z);")
                pointNormalLineString.append("\(pclPointNormalCloud.normals[currentPointsIdx].x);")
                pointNormalLineString.append("\(pclPointNormalCloud.normals[currentPointsIdx].y);")
                pointNormalLineString.append("\(pclPointNormalCloud.normals[currentPointsIdx].z)\n")
                
                framePointsNormalsString.append(pointNormalLineString)
                allPointsNormalsString.append(pointNormalLineString)
                
                currentPointsIdx += 1
            }
            do {
                try uploadTextFile(input: framePointsNormalsString, name: "Frame_index_\(frameIdx)")
            } catch {}
        }
        do {
            try uploadTextFile(input: allPointsNormalsString, name: "All_Points_and_Normals")
        } catch {}
    }
    
    private func constructSurfaceNode(pclMesh: PCLMesh) -> SCNNode {
        
        var vertices = [SCNVector3]()
        for i in 0..<pclMesh.numPoints {
            vertices.append(SCNVector3(x: Float(pclMesh.points[i].x), y: Float(pclMesh.points[i].y), z: Float(pclMesh.points[i].z)))
        }
        let vertexSource = SCNGeometrySource(vertices: vertices)
        
        var elements = [SCNGeometryElement]()
        for i in 0..<pclMesh.numFaces {
            let allPrimitives: [Int32] = [pclMesh.polygons[i].v1, pclMesh.polygons[i].v2, pclMesh.polygons[i].v3]
            let element = SCNGeometryElement(indices: allPrimitives, primitiveType: .triangles)
            elements.append(element)
        }
        
        surfaceGeometry = SCNGeometry(sources: [vertexSource], elements: elements)
        surfaceGeometry!.firstMaterial?.isDoubleSided = true;
        surfaceGeometry!.firstMaterial?.diffuse.contents = UIColor(displayP3Red: 135/255, green: 206/255, blue: 250/255, alpha: 1)
        surfaceGeometry!.firstMaterial?.lightingModel = .blinn
        return SCNNode(geometry: surfaceGeometry!)
    }
    
    @IBAction func exportSurface(sender: UIButton) {
        uploadFolder()
        
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
        tempFileURL.appendPathExtension("obj")
        fileManager.createFile(atPath: tempFileURL.path, contents: Data())
        
        // Export mesh to temporary file
        do {
            print(MDLAsset.canExportFileExtension("obj"))
            try mdlAsset.export(to: tempFileURL)
            
            // Read from file
            let plyString = try String(contentsOf: tempFileURL, encoding: .utf8)
            
            // Upload file
            do {
                try uploadTextFile(input: plyString, name: "SurfaceModel", fileExtension: "obj")
            } catch {
                showAlert(title: "Upload Error", message: "Make sure that the folder has been uploaded successfully and try again.")
                return
            }
            showAlert(title: "Sucessful Upload", message: "Mesh was uploaded to Google Drive Folder")

            // Discard file
            do {
                try fileManager.removeItem(at: tempFileURL)
            } catch {
                print("Temp file not deleted")
            }
            
        } catch {
            showAlert(title: "Error exporting to file", message: "")
            return
        }
    }
    
    @IBAction func uploadPointsTextFile(sender: UIButton) {
        uploadFolder()
        
        let input = createXyzRgbString(points: points, pointColors: colors)
        do {
            try uploadTextFile(input: input, name: "All_Points_and_Colors")
        } catch {
            showAlert(title: "Upload Error", message: "Make sure that the folder has been uploaded successfully and try again.")
        }
    }
    
    @IBAction func resetScene(sender: UIButton) {
        
        points = []
        colors = []
        pointCloudFrameSizes = []
        pointCloudFrameViewpoints = []
        
        surfaceParentNode.enumerateChildNodes { (node, stop) in node.removeFromParentNode() }
        pointsParentNode.enumerateChildNodes { (node, stop) in node.removeFromParentNode() }
        
        sceneView.debugOptions.remove(ARSCNDebugOptions.showFeaturePoints)
        sceneView.debugOptions.remove(ARSCNDebugOptions.showWorldOrigin)
        
        // Run the view's session
        sceneView.session.run(sessionConfiguration, options: [ARSession.RunOptions.resetTracking, ARSession.RunOptions.removeExistingAnchors])
        
        sceneView.debugOptions.insert(ARSCNDebugOptions.showFeaturePoints)
        sceneView.debugOptions.insert(ARSCNDebugOptions.showWorldOrigin)
    }
    
    @IBAction func toggleTorch(sender: UIButton) {
        guard let device = AVCaptureDevice.default(for: AVMediaType.video)
            else {return}
        
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
    
    @IBAction func showInfoPopup(sender: UIButton) {
        let title = "Info"
        let message = "Detected points are shown in yellow. Tap the screen to add currently detected ponts to the captured point cloud sample. Captured points are shown in white. Press Upload to upload a text file of the captured points to the associated Google Drive account."
        showAlert(title: title, message: message)
    }
    
    @IBAction func showOptionsPopup(sender: UIButton) {
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
    
    @IBAction func switchValueDidChange(sender:UISwitch!) {
        if (sender.isOn){
            isMultipartUploadOn = true
        }
        else {
            isMultipartUploadOn = false
        }
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
    
    // MARK: - Google Drive Helper Functions
    
    enum UploadError: Error {
        case fileUploadError
    }
    
    private func uploadTextFile(input: String, name: String, fileExtension: String = "txt") throws {
        let fileData = input.data(using: .utf8)!
    
        let metadata = GTLRDrive_File()
        metadata.parents = [folderID]
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
    
    private func uploadImageFile(image: UIImage, name: String) {
        
        let content = image
        let mimeType = "image/jpeg"
        
        let metadata = GTLRDrive_File()
        metadata.parents = [folderID]
        metadata.name = name + ".jpg"
        
        guard let data = UIImageJPEGRepresentation(content, CGFloat(imageQuality)) else {
            return
        }
        
        let uploadParameters = GTLRUploadParameters(data: data, mimeType: mimeType)
        uploadParameters.shouldUploadWithSingleRequest = true

        let query = GTLRDriveQuery_FilesCreate.query(withObject: metadata, uploadParameters: uploadParameters)
        self.service.executeQuery(query, completionHandler: {(ticket:GTLRServiceTicket, object:Any?, error:Error?) in
            if error == nil {
                print("Image File Success")
            }
            else {
                print("An error occurred: \(String(describing: error))")
                self.showAlert(title: "Image Upload Error", message: "Make sure that the folder has been uploaded successfully and try again.")
            }
            self.pendingImageUploads -= 1
        })
    }
    
    private func uploadFolder() {
        if hasFolderBeenUploaded {
            return
        }
        
        let dateFormatter = DateFormatter()
        dateFormatter.dateStyle = DateFormatter.Style.short
        dateFormatter.timeStyle = DateFormatter.Style.short
        let timeDateString = dateFormatter.string(from: Date())
        let name = timeDateString
        
        let metadata = GTLRDrive_File()
        metadata.name = name
        metadata.mimeType = "application/vnd.google-apps.folder"
        
        let query = GTLRDriveQuery_FilesCreate.query(withObject: metadata, uploadParameters: nil)
        self.service.executeQuery(query, completionHandler: {(ticket:GTLRServiceTicket, object: Any?, error:Error?) in
            if error == nil {
                let file = object as? GTLRDrive_File
                self.folderID = (file?.identifier)!
                self.hasFolderBeenUploaded = true
                print("Folder Upload Success")
                self.showAlert(title: "Folder Upload Success", message: "You may now tap to add images to the folder.")
            }
            else {
                print("An error occurred: \(String(describing: error))")
            }
        })
    }
    
    
    // MARK: - Text File Formatting Helper Functions

    private func createXyzString(points: [float3]) -> String {
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
    
    private func createXyzRgbString(points: [float3], pointColors: [UIColor?]) -> String {
        
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
    
    
    // MARK: - Image Metadata Helper Functions
    
    private func capturePointColors(currentPoints: [float3]) -> [UIColor?] {
        
        var pointColors: [UIColor?] = []
        
        if let frame = sceneView.session.currentFrame {
            do {
                let capturedImageSampler = try CapturedImageSampler(frame: frame)
                
                for point in currentPoints {
                    let point2DPos = sceneView.projectPoint(SCNVector3(point))
                    if let pointColor = capturedImageSampler.getColor(atX: CGFloat(point2DPos.x) / sceneView.frame.maxX, y: CGFloat(point2DPos.y) / sceneView.frame.maxY) {
                        pointColors.append(pointColor)
                    } else {
                        pointColors.append(nil)
                    }
                }
                return pointColors
            } catch {
                print("Error")
                return pointColors
            }
        }
        return pointColors
    }
    
    private func determineImageOrientation() -> UIImageOrientation {
        switch UIApplication.shared.statusBarOrientation {
        case .landscapeLeft:
            return UIImageOrientation.down
        case .landscapeRight:
            return UIImageOrientation.up
        case .portrait:
            return UIImageOrientation.right
        case .unknown:
            return UIImageOrientation.right
        case .portraitUpsideDown:
            return UIImageOrientation.left
        }
    }
    
    private func projectPoint2DPositions(currentPoints: [float3]) -> [float3] {
        var point2DPositions: [float3] = []
        
        for point in currentPoints {
            var point2DPos = sceneView.projectPoint(SCNVector3(point))
            point2DPos.x /= Float(sceneView.frame.width)
            point2DPos.y /= Float(sceneView.frame.height)
            point2DPos.x = (point2DPos.x * 2) - 1
            point2DPos.y = (point2DPos.y * 2) - 1
            point2DPositions.append(float3(point2DPos))
        }
        return point2DPositions
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
        
        if let pointMaterial = pointMaterial {
            sphere.firstMaterial = pointMaterial
        }
        let sphereNode = SCNNode(geometry: sphere)
        sphereNode.orientation = (sceneView.pointOfView?.orientation)!
        sphereNode.pivot = SCNMatrix4MakeRotation(-Float.pi / 2, 0, 1, 0)
        sphereNode.position = SCNVector3(position)
        pointsParentNode.addChildNode(sphereNode)
    }
    
 
    // MARK: - ARSCNViewDelegate
    
//    func renderer(_ renderer: SCNSceneRenderer, didAdd node: SCNNode, for anchor: ARAnchor) {
        // This visualization covers only detected planes.
//        if let planeAnchor = anchor as? ARPlaneAnchor {
//            print("Plane anchor!")
//            // Create a SceneKit plane to visualize the node using its position and extent.
//            let plane = SCNPlane(width: CGFloat(planeAnchor.extent.x), height: CGFloat(planeAnchor.extent.z))
//            let planeNode = SCNNode(geometry: plane)
//            planeNode.position = SCNVector3Make(planeAnchor.center.x, 0, planeAnchor.center.z)
//
//            // SCNPlanes are vertically oriented in their local coordinate space.
//            // Rotate it to match the horizontal orientation of the ARPlaneAnchor.
//            planeNode.transform = SCNMatrix4MakeRotation(-Float.pi / 2, 1, 0, 0)
//
//            // ARKit owns the node corresponding to the anchor, so make the plane a child node.
//            node.addChildNode(planeNode)
//        }
//    }
    
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
        let currentPoints = rawFeaturePoints.points
        
        let pointColors = capturePointColors(currentPoints: currentPoints)
        colors += pointColors // Add current colors to global list
    
        let camera = sceneView.session.currentFrame?.camera
   
//        if isMultipartUploadOn {
//            uploadFolder() // Only uploads if folder hasn't been uploaded
//
//            // Create corresponding file name for different types of uploads
//            let dateFormatter = DateFormatter()
//            dateFormatter.dateFormat = "H:m:ss:SSSS"
//            let timeString = dateFormatter.string(from: Date())
//
//            // Upload Image
//            if let frame = sceneView.session.currentFrame {
//                let imageWithCVPixelBuffer = frame.capturedImage
//                let ciImage = CIImage(cvPixelBuffer: imageWithCVPixelBuffer)
//                let tempContext = CIContext()
//                let videoImage = tempContext.createCGImage(ciImage, from: CGRect(x: 0, y: 0, width: CGFloat(CVPixelBufferGetWidth(imageWithCVPixelBuffer)), height: CGFloat(CVPixelBufferGetHeight(imageWithCVPixelBuffer))))
//                let image = UIImage(cgImage: videoImage!, scale: 1.0, orientation: determineImageOrientation())
//
//                pendingImageUploads += 1
//                uploadImageFile(image: image, name: "Photo_\(timeString)")
//            }
            
//            // Upload Points and Colors text file
//            uploadTextFile(input: createXyzRgbString(points: currentPoints, pointColors: pointColors), name: "Points_and_Colors_\(timeString)")
            
//            // Upload 2D point positions
//            let point2DPositions = projectPoint2DPositions(currentPoints: currentPoints)
//            uploadTextFile(input: createXyzString(points: point2DPositions), name: "2D_Point_Positions_\(timeString)")
            
//            // Upload camera info text file
//            let transform: String = "Transform: " + (camera?.transform.debugDescription)!
//            let eulerAngles: String = "Euler Angles: " + (camera?.eulerAngles.debugDescription)!
//            let intrinsics: String = "Intrinsics: " + (camera?.intrinsics.debugDescription)!
//            uploadTextFile(input: transform + "\n" + eulerAngles + "\n" + intrinsics, name: "6DOF_\(timeString)")
//        }
        
        // Display points
        var i = 0
        for rawPoint in currentPoints {
            if i % addPointRatio == 0 {
                addPointToView(position: rawPoint)
            }
            i += 1
        }
        points += currentPoints
        pointCloudFrameSizes.append(Int32(currentPoints.count))
        
        // Add view point
        if let transform = camera?.transform {
            let position = SCNVector3(
                transform.columns.3.x,
                transform.columns.3.y,
                transform.columns.3.z
            )
            pointCloudFrameViewpoints.append(position)
        }
    }
}
