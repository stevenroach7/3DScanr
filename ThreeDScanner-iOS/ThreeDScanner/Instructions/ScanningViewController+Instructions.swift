//
//  ScanningViewController+Instructions.swift
//  ThreeDScanner
//
//  Created by Steven Roach on 5/24/18.
//  Copyright © 2018 Steven Roach. All rights reserved.
//

import Foundation
import Instructions

extension ScanningViewController: CoachMarksControllerDataSource, CoachMarksControllerDelegate {
    
    func numberOfCoachMarks(for coachMarksController: CoachMarksController) -> Int {
        return 7
    }
    
    func coachMarksController(_ coachMarksController: CoachMarksController, coachMarkAt index: Int) -> CoachMark {
        var coachMark = coachMarksController.helper.makeCoachMark()
        
        switch(index) {
        case 0:
            coachMark = coachMarksController.helper.makeCoachMark(for: self.signInButton)
            coachMark.arrowOrientation = .top
            signInButton.isHidden = false
            signOutButton.isHidden = true
            capturePointsButton.isHidden = false
            pauseCapturePointsButton.isHidden = true
            reconstructButton.isHidden = true
            exportButton.isHidden = true
            resumeScanningButton.isHidden = true
        case 1:
            coachMark = coachMarksController.helper.makeCoachMark(for: self.capturePointsButton)
            coachMark.arrowOrientation = .bottom
            signInButton.isHidden = true
            signOutButton.isHidden = false
        case 2:
            coachMark = coachMarksController.helper.makeCoachMark(for: self.pauseCapturePointsButton)
            coachMark.arrowOrientation = .bottom
            capturePointsButton.isHidden = true
            pauseCapturePointsButton.isHidden = false
            reconstructButton.isHidden = false
        case 3:
            coachMark = coachMarksController.helper.makeCoachMark(for: self.reconstructButton)
            coachMark.arrowOrientation = .bottom
        case 4:
            coachMark = coachMarksController.helper.makeCoachMark(for: self.exportButton)
            coachMark.arrowOrientation = .bottom
            pauseCapturePointsButton.isHidden = true
            reconstructButton.isHidden = true
            exportButton.isHidden = false
            resumeScanningButton.isHidden = false
        case 5:
            coachMark = coachMarksController.helper.makeCoachMark(for: self.resumeScanningButton)
            coachMark.arrowOrientation = .bottom
        case 6:
            coachMark = coachMarksController.helper.makeCoachMark(for: self.capturePointsButton)
            coachMark.arrowOrientation = .bottom
            capturePointsButton.isHidden = false
            exportButton.isHidden = true
            resumeScanningButton.isHidden = true
        default:
            return coachMark
        }
        return coachMark
    }
    
    func coachMarksController(_ coachMarksController: CoachMarksController, coachMarkViewsAt index: Int, madeFrom coachMark: CoachMark) -> (bodyView: CoachMarkBodyView, arrowView: CoachMarkArrowView?) {
        
        let coachMarkBodyView = TransparentCoachMarkBodyView()
        var coachMarkArrowView: CoachMarkArrowView? = nil
        
        switch(index) {
        case 0:
            coachMarkBodyView.hintLabel.text = InstructionsText.text1
            coachMarkArrowView = TransparentCoachMarkArrowView(orientation: .topRight)
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
        case 6:
            coachMarkBodyView.hintLabel.text = InstructionsText.text7
            coachMarkArrowView = TransparentCoachMarkArrowView(orientation: .bottomRight)
        default: break
        }
        return (bodyView: coachMarkBodyView, arrowView: coachMarkArrowView)
    }
    
    func coachMarksController(_ coachMarksController: CoachMarksController, constraintsForSkipView skipView: UIView, inParent parentView: UIView) -> [NSLayoutConstraint]? {
        
        var constraints: [NSLayoutConstraint] = []
        constraints.append(contentsOf: NSLayoutConstraint.constraints(withVisualFormat: "V:|-50-[skipView(==44)]", options: NSLayoutFormatOptions(rawValue: 0), metrics: [:], views: ["skipView": skipView]))
        return constraints
    }
    
    func coachMarksController(_ coachMarksController: CoachMarksController, didEndShowingBySkipping skipped: Bool) {
        signOutButton.isHidden = !isUserSignedOn
        signInButton.isHidden = isUserSignedOn
        signOutButton.isHidden = !isUserSignedOn
        signInButton.isHidden = isUserSignedOn
        capturePointsButton.isHidden = false
        pauseCapturePointsButton.isHidden = true
        reconstructButton.isHidden = true
        exportButton.isHidden = true
        resumeScanningButton.isHidden = true
    }
}
