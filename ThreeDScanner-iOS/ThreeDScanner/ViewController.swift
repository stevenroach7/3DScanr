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

class ViewController: UIViewController, ARSCNViewDelegate {

    @IBOutlet var sceneView: ARSCNView!
    
    var points: [vector_float3] = []
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        // Set the view's delegate
        sceneView.delegate = self
        
        addCopyButton()
        addResetButton()
    }
    
    @IBAction func showPointsButtonTapped(sender: UIButton) {
        print(points.description)
//        UIPasteboard.general.string = points.description
    }
    
    func addCopyButton() {
        let copyButton = UIButton()
        view.addSubview(copyButton)
        copyButton.translatesAutoresizingMaskIntoConstraints = false
        copyButton.setTitle("Copy points", for: .normal)
        copyButton.setTitleColor(UIColor.red, for: .normal)
        copyButton.backgroundColor = UIColor.white.withAlphaComponent(0.4)
        copyButton.addTarget(self, action: #selector(showPointsButtonTapped(sender:)) , for: .touchUpInside)
        
        // Contraints
        copyButton.bottomAnchor.constraint(equalTo: view.bottomAnchor, constant: -8.0).isActive = true
        copyButton.centerXAnchor.constraint(equalTo: view.centerXAnchor, constant: 0.0).isActive = true
        copyButton.heightAnchor.constraint(equalToConstant: 50)
    }
    
    @IBAction func resetPointsButtonTapped(sender: UIButton) {
        points = []
    }
    
    func addResetButton() {
        let resetButton = UIButton()
        view.addSubview(resetButton)
        resetButton.translatesAutoresizingMaskIntoConstraints = false
        resetButton.setTitle("Reset points", for: .normal)
        resetButton.setTitleColor(UIColor.red, for: .normal)
        resetButton.backgroundColor = UIColor.white.withAlphaComponent(0.4)
        resetButton.addTarget(self, action: #selector(resetPointsButtonTapped(sender:)) , for: .touchUpInside)
        
        // Contraints
        resetButton.bottomAnchor.constraint(equalTo: view.bottomAnchor, constant: -8.0).isActive = true
        resetButton.rightAnchor.constraint(equalTo: view.rightAnchor, constant: -8.0).isActive = true
        resetButton.heightAnchor.constraint(equalToConstant: 50)
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        
        // Create a session configuration
        let configuration = ARWorldTrackingConfiguration()

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
    
    func addPointToView(position: vector_float3) {
        let sphere = SCNSphere(radius: 0.00033)
        let sphereNode = SCNNode(geometry: sphere)
        sphereNode.position = SCNVector3(position)
        sceneView.scene.rootNode.addChildNode(sphereNode)
    }
}
