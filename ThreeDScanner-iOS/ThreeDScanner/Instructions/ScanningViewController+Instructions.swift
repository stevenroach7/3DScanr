//
//  ScanningViewController+Instructions.swift
//  ThreeDScanner
//
//  Created by Steven Roach on 5/24/18.
//  Copyright © 2018 Steven Roach. All rights reserved.
//

import Foundation
import Instructions
import ARKit

extension ScanningViewController: CoachMarksControllerDataSource, CoachMarksControllerDelegate {
    
    func numberOfCoachMarks(for coachMarksController: CoachMarksController) -> Int {
        return 6
    }
    
    func coachMarksController(_ coachMarksController: CoachMarksController, coachMarkAt index: Int) -> CoachMark {
        var coachMark = coachMarksController.helper.makeCoachMark()
        
        switch(index) {
        case 0:
            coachMark = coachMarksController.helper.makeCoachMark(for: self.capturePointsButton)
            coachMark.arrowOrientation = .bottom
            isCapturingPoints = false
            capturePointsButton.isSelected = false
            reconstructButton.isEnabled = false
            exportButton.isEnabled = false
        case 1:
            coachMark = coachMarksController.helper.makeCoachMark(for: self.capturePointsButton)
            coachMark.arrowOrientation = .bottom
            capturePointsButton.isSelected = true
            reconstructButton.isEnabled = true
        case 2:
            coachMark = coachMarksController.helper.makeCoachMark(for: self.reconstructButton)
            coachMark.arrowOrientation = .bottom
            capturePointsButton.isSelected = false
        case 3:
            coachMark = coachMarksController.helper.makeCoachMark(for: self.exportButton)
            coachMark.arrowOrientation = .bottom
            exportButton.isEnabled = true
        case 4:
            coachMark = coachMarksController.helper.makeCoachMark(for: self.capturePointsButton)
            coachMark.arrowOrientation = .bottom
        case 5:
            coachMark = coachMarksController.helper.makeCoachMark(for: self.capturePointsButton)
            coachMark.arrowOrientation = .bottom
        default:
            return coachMark
        }
        return coachMark
    }
    
    func coachMarksController(_ coachMarksController: CoachMarksController,
                              coachMarkViewsAt index: Int,
                              madeFrom coachMark: CoachMark) -> (bodyView: CoachMarkBodyView, arrowView: CoachMarkArrowView?) {
        
        let coachMarkBodyView = TransparentCoachMarkBodyView()
        var coachMarkArrowView: CoachMarkArrowView? = nil
        
        switch(index) {
        case 0:
            coachMarkBodyView.hintLabel.text = InstructionsText.text1
            coachMarkArrowView = TransparentCoachMarkArrowView(orientation: .bottomRight)
        case 1:
            coachMarkBodyView.hintLabel.text = InstructionsText.text2
            coachMarkArrowView = TransparentCoachMarkArrowView(orientation: .bottomRight)
        case 2:
            coachMarkBodyView.hintLabel.text = InstructionsText.text3
            coachMarkArrowView = TransparentCoachMarkArrowView(orientation: .bottomRight)
        case 3:
            coachMarkBodyView.hintLabel.text = InstructionsText.text4
            coachMarkArrowView = TransparentCoachMarkArrowView(orientation: .bottomRight)
        case 4:
            coachMarkBodyView.hintLabel.text = InstructionsText.text5
            coachMarkArrowView = TransparentCoachMarkArrowView(orientation: .bottomRight)
        case 5:
            coachMarkBodyView.hintLabel.text = InstructionsText.text6
            coachMarkArrowView = TransparentCoachMarkArrowView(orientation: .bottomRight)
        default: break
        }
        return (bodyView: coachMarkBodyView, arrowView: coachMarkArrowView)
    }
    
    func coachMarksController(_ coachMarksController: CoachMarksController,
                              constraintsForSkipView skipView: UIView,
                              inParent parentView: UIView) -> [NSLayoutConstraint]? {
        
        var constraints: [NSLayoutConstraint] = []
        constraints.append(skipView.centerYAnchor.constraint(equalTo: parentView.centerYAnchor, constant: -20.0))
        constraints.append(skipView.centerXAnchor.constraint(equalTo: parentView.centerXAnchor, constant: 0.0))
        return constraints
    }
    
    func coachMarksController(_ coachMarksController: CoachMarksController,
                              configureOrnamentsOfOverlay overlay: UIView) {
        
        let tapToContinueLabel = UITextView()
        overlay.addSubview(tapToContinueLabel)
        tapToContinueLabel.text = "Tap anywhere to continue"
        tapToContinueLabel.backgroundColor = UIColor.clear
        tapToContinueLabel.textColor = UIColor.white
        tapToContinueLabel.font = UIFont(name: InstructionsText.fontString, size: 25)
        tapToContinueLabel.isScrollEnabled = false
        tapToContinueLabel.textAlignment = .justified
        tapToContinueLabel.layoutManager.hyphenationFactor = 0.0
        tapToContinueLabel.isEditable = false
        tapToContinueLabel.translatesAutoresizingMaskIntoConstraints = false
        tapToContinueLabel.isUserInteractionEnabled = false
        
        tapToContinueLabel.centerYAnchor.constraint(equalTo: overlay.centerYAnchor, constant: -50.0).isActive = true
        tapToContinueLabel.centerXAnchor.constraint(equalTo: overlay.centerXAnchor, constant: 0.0).isActive = true
    }
    
    func coachMarksController(_ coachMarksController: CoachMarksController, didEndShowingBySkipping skipped: Bool) {
        updateScanningViewState()
    }
}
