<p align="center">
  <img src="https://raw.githubusercontent.com/lasarobotics/ftcvision/img/logo.png?raw=true" alt="FTC-Sees!"/>
</p>
This is a fork of the FTCVision project. For the original project see here.
https://github.com/lasarobotics/FTCVision

# FTC Vision Library [![Build Status](https://travis-ci.org/lasarobotics/FTCVision.svg?branch=master)](https://travis-ci.org/lasarobotics/FTCVision) [![Documentation Status](https://img.shields.io/badge/documentation-0.9.0%20(up%20to%20date)-blue.svg)](http://ftcvision.lasarobotics.org)
Computer Vision library for FTC based on OpenCV, featuring **beacon color and position detection**, as well as an easy-to-use `VisionOpMode` format and many additional detection features planned in the future.

## Installing from Scratch

1. Clone FTCVision into a clean directory (outside your robot controller app) using the following command: `git clone --depth=1 https://github.com/lasarobotics/ftcvision`.
2. Open the FTCVision project using Android Studio
3. Copy your OpModes from your robot controller directory into the appropriate directory within `ftc-robotcontroller`. Then, modify the `FtcOpModeRegister` appropriately to add your custom OpModes.
4. Before running the app for the first time, install the "OpenCV Manager" from the Google Play Store to enable Vision processing.
5. Run and test the code! Let us know if you encounter any difficulties.
6. You can now write your own `VisionOpMode`!

## Installing into Existing Project

- Clone FTCVision into a clean directory (outside your robot controller app) using the following command: `git clone --depth=1 https://github.com/lasarobotics/ftcvision`.
- Navigate to the FTCVision directory that you just cloned and copy the `ftc-visionlib` and `opencv-java` folders into your existing robot controller app.
- Open your robot controller app in Android Studio. Make sure you have the `Project` mode selected in the project browser window (so you can see all of the files in your project).
- Find your `settings.gradle` file and append the following two lines:
```
include ':opencv-java'
include ':ftc-visionlib'
```
- Find the `AndroidManifest.xml` under your `ftc-robotcontroller` folder, sometimes named `sample` or similar.
- Insert the following `uses-permission` tag in the appropriate location (look at the rest of the file for context).
```
<uses-permission android:name="android.permission.CAMERA" android:required="true" />
```
- Find your `build.gradle` in the parent folder of `AndroidManifest.xml` and insert the following line under `dependencies`:
```
compile project(':ftc-visionlib')
compile project(':opencv-java')
```
- Update Gradle configuration by pressing the green "Sync Project with Gradle Files" button in the header (this may take a minute)
- Copy in Vision opmodes (those that end in `VisionSample.java`, located in `[vision-root]/ftc-robotcontroller/src/main/java/com/qualcomm/ftcrobotcontroller/opmodes`) from the FTCVision directory into your opmode directory.
- Before running the app for the first time, install the "OpenCV Manager" from the Google Play Store to enable Vision processing.
- Run and test the code! Let us know if you encounter any difficulties.
- You can now write your custom `VisionOpMode`!

## Status
This library is currently under insanely active development. We're in the **Beta** phase right now. If you have any questions or would like to help, send a note to `smo-key` (contact info on profile). Thank you!

## Documentation [![Documentation Status](https://img.shields.io/badge/documentation-0.9.0%20(up%20to%20date)-blue.svg)](http://ftcvision.lasarobotics.org)

Documentation for the stable library is available at http://ftcvision.lasarobotics.org.

## Does it work?

**Yes!** FTCVision can detect a beacon 0.5-4 feet away with 65% accuracy in 0.3 seconds. Here are some pictures. :smiley:

#### Accuracy Test
![Can it detect the beacon?](https://raw.githubusercontent.com/lasarobotics/ftcvision/img/test2.png)

#### Distance Test
![A test from 8 feet away](https://raw.githubusercontent.com/lasarobotics/ftcvision/img/test1.png)

#### Basic Analysis Demo
![FAST isn't the greatest](https://raw.githubusercontent.com/lasarobotics/ftcvision/img/analysisdemo.gif)

#### Ambient Analysis (Color and Rotation Detection) Test
![A test from 8 feet away](https://raw.githubusercontent.com/lasarobotics/ftcvision/img/test3.gif)

#### Analysis Methods
![FAST vs. COMPLEX](https://raw.githubusercontent.com/lasarobotics/ftcvision/img/methods.png)

## Goals
- Locate the lit target (the thing with two buttons) within the camera viewfield
- Move the robot to the lit target, while identifying the color status of the target
- Locate the button of the target color and activate it

## Progress
- Beacon located successfully with automated environmental and orientation tuning.
- A competition-proof `OpMode` scheme created so that the robot controller does not need to be modified to use the app.
- Now supports nearly every phone since Android 4.2, including both the ZTE Speed and Moto G.
- All initial goals complete - now tweaking beacon detection and preparing for something more...
