# eVistDrive Firmware

**eVistDrive** is an open-source torque-sensing e-bike motor control platform. This repository holds the motor controller firmware for the hubmotor controller CRA101C with the GD32F303RCT6 processor.

eVistDrive combines the proven motor control architecture of **EBiCS** with rider-assistance concepts inspired by open-source **TSDZ** firmware projects. It is an independent open-source fork — **not** an official product of the original EBiCS or TSDZ developers.

> eVistDrive is derived from the EBiCS motor controller firmware and incorporates concepts inspired by open-source TSDZ firmware projects. Original copyright notices and licenses are preserved.

This project is under construction. All you are doing with this project is on your own risk. The authors do not accept any liability for damage to property or personal injury!  
It is strongly recommented to use a fuse between controller and battery. 
Basic functions are implemented, a very first release is published. 
The bin file can be flashed with the [Open Bafang Canable Tool](https://github.com/mdi-9/bafang_canable_pro/releases).  

The canable tool can be used to setup most relevant parameters, but some fields have a different meaning than in the original Bafang firmware and some fields have no function at all yet.  

Attention: the button "Torquesensor Calibration" is used to reset all parameters to their default values!  

For discussion visit the [Endless Sphere forum](https://endless-sphere.com/sphere/threads/foc-open-source-firmware-for-bafang-can-bus-controllers-with-gd32f303-processor.128869/)  

![ElectricParameters](/documentation/ElectricParameters.JPG)  
![BatteryParameters](/documentation/BatteryParameters.JPG)  
![MechanicalParameters](/documentation/MechanicalParameters.JPG)  
![DrivingParameters](/documentation/DrivingParameters.JPG)  
![ThrottleParameters](/documentation/ThrottleParameters.JPG)  
![SpeedSettings](/documentation/SpeedSettings.JPG)  
![AssistFullTab](/documentation/AssistFullTab.JPG)  

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
