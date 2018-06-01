# 3DScanR

[![GitHub license](https://img.shields.io/github/license/mashape/apistatus.svg)](https://github.com/3DScanr/LICENSE)

3D Scan, view, and export in seconds with just your iPhone or iPad!

This project uses [ARKit](https://developer.apple.com/arkit/) to detect feature points on surfaces. These points are then be used to reconstruct a 3D surface using the [Point Cloud Library](http://pointclouds.org/). This surface can be viewed with ARKit and exported to a user's Google Drive account.  

To try it out, [download 3DScanR from the App Store](https://itunes.apple.com/us/app/3dscanr/id1328996034?mt=8).  

OR 

Clone the repository and open the iOS project in XCode. Building the PCL libraries needed to run surface reconstruction on the device was a huge pain. Luckily, I've included the static library files in the PCL_Build_Artifacts directory. Just drag these files into the Linked Frameworks and Libraries list which is in the General tab of the ThreeDScanner target. If you run into problems, please open an issue.

Then run the app on an ARKit compatible device. Happy Scanning!
