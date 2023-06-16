# 3D LiDAR scanner v1 - With Adafruit Mini Pan-Tilt and ESP32-C3-DevKitM

This repository demonstrates a homemade 3D LiDAR scanner made out of a [Benewake TF-Mini](https://www.gotronic.fr/art-capteur-de-distance-lidar-tf-mini-27615.htm) LiDAR sensor and [Adafruit Mini Pan-Tilt Kit](https://www.adafruit.com/product/1967). This scanner can produce 3D cloud points of spaces and was made to extract 3D point clouds of objects by subtracting two scans : one of the background and one of the object in the space. The two steps (scan of the background and scan of the background  and the object) can be triggered with a physical button. Once the object cloudpoint is extracted, we planned to determine its height and gravity center 3D position to place it in the center of the space (not yet implemented).


|<img height="300" src="https://github.com/CLICKBE/MWE-scanner_stepper/assets/2494294/8350b0a7-daef-4660-8843-7ecf97d9d9a3" alt="scannerLidar-stepper"> | <img height="300" src="https://github.com/CLICKBE/MWE-scanner_stepper/assets/2494294/b51f2bb5-2d3f-4970-921e-cd1419681865" alt="scan3D-exemple">|
| :---: | :---: |
| The 3D LiDAR scanner | Example of 3D visualisation of the scanning |

## Files in the repository
- src/* : All the files use to run the program
- fritzing : [Fritzing](https://fritzing.org/) file used to draw the connections picture

## Using the 3D LiDAR scanner v1 - With Adafruit Mini Pan-Tilt

### Code

This code has been developped with VSCode and PlatformIO backend. This repo therefore reproduced the fodler structure used in such project. Once you opened VSCode, open PlatformIO home page and hit Open Project button. Then, select the folder of the repository.

If you don't know anything about VSCode and PlatformIO, here's a link to set you up : [https://platformio.org/install/ide?install=vscode](https://platformio.org/install/ide?install=vscode).

### Parts
See `3DLiDAR_scanner-v1-BOM.csv`at the root of this repository.

Power is handled by an external power supply delivering 9V, 1.5A. 

### Connection

<img height = 500 src=https://github.com/CLICKBE/3DLidarScanner-v1/assets/2494294/4b8f6705-4c27-4613-b089-9a572626b98d alt = "3DLidarScanner-v1">

### Library needed : 
- TFMPlus : [https://github.com/budryerson/TFMini-Plus][(https://github.com/budryerson/TFMini-Plus), no licence so author's right. 
- ESP32Servo : [https://github.com/madhephaestus/ESP32Servo](https://github.com/madhephaestus/ESP32Servo), GPL2.1 

Those libraries are not included in this git repository but are referenced in the platformio.ini file and therefore should directly be downloaded by PlatformIO extension of VSCode when building the code. As none of those libraries are made available with persmissive open-source licence type, you have to contact the libraries authors if you want to include them in a product. 


## 3D Visualization :
To perform a quick 3D visualization based on the serial data you can use the Processing script developed by Dana Peters : [LidarScanner.pde](https://drive.google.com/file/d/1D5wfzA8i0Pzh4qe-1skmpnqmhrvaq9d3/view?usp=drive_web) who also developped a [3D LiDAR scanner](https://www.qcontinuum.org/lidar-scanner).

In order to use this script you will need free [Processing](https://processing.org/) software.


## License
 © 2022 – CLICK - Université de Mons

3D LiDAR scanner with stepper motors – CLICK - UMONS (Loïc Reboursière) is free software: you can redistribute it and/or modify it under the terms of the Apache License Version 2.0. This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the Apache License Version 2.0 for more details.
You should have received a copy of the Apache License Version 2.0 along with this program.  If not, see http://www.apache.org/licenses/
Each use of this software must be attributed to University of MONS – CLICK (Loïc Reboursière).
Any other additional authorizations may be asked to avre@umons.ac.be.

## Legal Notices
This work was produced as part of the FEDER Digistorm project, co-financed by the European Union and the Wallonia Region.

![Logo FEDER-FSE](https://www.enmieux.be/sites/default/files/assets/media-files/signatures/vignette_FEDER%2Bwallonie.png)
