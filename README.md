# 3DScanr

[![GitHub license](https://img.shields.io/github/license/mashape/apistatus.svg)](https://github.com/3DScanr/LICENSE)

This project uses ARKit to detect feature points on surfaces. These points are then be used to reconstruct a 3D surface. 

To try it out, clone the repository and open the iOS project in XCode. Then run the app on an ARKit compatible device. Next, sign into your google drive account and scan an object by touching the screen and capturing the object from different angles. A point cloud made up of white captured points will appear. This point cloud is used to reconstruct a surface when the "Reconstruct" button is pressed. Once you are happy with your reconstruction, press the "Export" button to upload an stl file representing the surface to your Google Drive. Happy Scanning!
