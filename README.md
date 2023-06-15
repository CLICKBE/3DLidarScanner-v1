# 3D LiDAR scanner v1 - With Adafruit Mini Pan-Tilt and ESP32-C3-DevKitM

This repository demonstrates a homemade 3D LiDAR scanner made out of a [Benewake TF-Mini](https://www.gotronic.fr/art-capteur-de-distance-lidar-tf-mini-27615.htm) LiDAR sensor and [Adafruit Mini Pan-Tilt Kit](https://www.adafruit.com/product/1967). This scanner can produce 3D cloud points of spaces and was made to extract 3D point clouds of objects by subtracting two scans : one of the background and one of the object in the space. Once the cloudpoint is extracted, we planned to determine its height and gravity center 3D position to place it in the center of the space (not yet implemented).


|<img height="300" src="https://github.com/CLICKBE/MWE-scanner_stepper/assets/2494294/8350b0a7-daef-4660-8843-7ecf97d9d9a3" alt="scannerLidar-stepper"> | <img height="300" src="https://github.com/CLICKBE/MWE-scanner_stepper/assets/2494294/b51f2bb5-2d3f-4970-921e-cd1419681865" alt="scan3D-exemple">|
| :---: | :---: |
| The 3D LiDAR scanner| Example of 3D visualisation of the scanning |

## Files in the repository
- src/* : All the files use to run the program
- data/index.htm : The webpage of the server runing on the ESP32
- fritzing : [Fritzing](https://fritzing.org/) file used to draw the connections picture

## Using the 3D LiDAR scanner v1 - With Adafruit Mini Pan-Tilt

### Parts
See `3DLiDAR_scanner-v1-BOM.csv`at the root of this repository.

Power is handled by an external power supply delivering 9V, 1.5A. 

### Connection

![3DLidarScanner with stepper connections](https://github.com/CLICKBE/3DLidarScanner-v2/blob/main/fritzing/3DLidarScanner-v2-connections.png?raw=true)

### Library needed : 
- TFMPlus : [https://github.com/budryerson/TFMini-Plus][(https://github.com/budryerson/TFMini-Plus). 
- AsyncTCP for ESP32 : [https://github.com/esphome/AsyncTCP](https://github.com/esphome/AsyncTCP), LGPL-3.9 license
- ESPAsyncWebServer : [https://github.com/esphome/ESPAsyncWebServer](https://github.com/esphome/ESPAsyncWebServer), author's right licence

Those libraries are not included in this git repository but are referenced in the platformio.ini file and therefore should directly be downloaded by PlatformIO extension of VSCode when building the code.

### Upload the server file to the ESP32

### Scanner serial protocol
This protocol is the one which was used in the [3D LiDAR Scanner-v2](https://github.com/CLICKBE/3DLidarScanner-v2). Because the Arduino Uno used in the 2nd version did not have enough memory to store the scanning data, they were sent on the fly through serial protocol. As a matter This protocol has been integrated afterwards in this version of the scanner. 

Once the Arduino script is uploaded and running onto the Arduino Uno board, the scanner can be control through serial protocol at 115200 bauds. The following commands are available : 

- s : perform 2D scan
- y : perform 1D scan (vertical)
- i : display scanner setup info
- v : followed by an integer value sets the vertical step (e.g. `v 1`)
- h : followed by an integer value sets the horizontal step (e.g. `h 1`)
- p : scan reboot
   
Once the scan is launched (through s or y), it performs the XYZ coordinates conversion and ouputs all of the data throught serial port in the following manner : 

`x y z h_idx v_idx distance `

With : 
- x : x coordinate in 3D space
- y : y coordinate in 3D space
- z : z coordinate in 3D space
- h_idx : horizontal index of the scan
- v_idx vertical index of the scan
- distance : data coming from TF-Mini-s LiDAR sensor

## Webserver

The 


## 3D Visualization

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
