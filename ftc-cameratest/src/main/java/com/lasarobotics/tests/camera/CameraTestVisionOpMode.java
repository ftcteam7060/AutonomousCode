package com.lasarobotics.tests.camera;

import android.util.Log;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.detection.ColorBlobDetector;
import org.lasarobotics.vision.detection.objects.Contour;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.image.Drawing;
import org.lasarobotics.vision.opmode.TestableVisionOpMode;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.lasarobotics.vision.util.color.ColorGRAY;
import org.lasarobotics.vision.util.color.ColorRGBA;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.lasarobotics.vision.ftc.resq.Constants;

import java.util.LinkedList;
import java.util.List;

/**
 * Vision OpMode run by the Camera Test Activity
 * Use TestableVisionOpModes in testing apps ONLY (but you can easily convert between opmodes just by changingt t
 */
public class CameraTestVisionOpMode extends TestableVisionOpMode {
    LinkedList<Float> blueCenterPoints;
    @Override
    public void init() {
        super.init();
        blueCenterPoints = new LinkedList<Float>();
        //Set the camera used for detection
        this.setCamera(Cameras.PRIMARY);
        //Set the frame size
        //Larger = sometimes more accurate, but also much slower
        //For Testable OpModes, this might make the image appear small - it might be best not to use this
        this.setFrameSize(new Size(900, 900));

        //Enable extensions. Use what you need.
        enableExtension(Extensions.BEACON);     //Beacon detection
        enableExtension(Extensions.ROTATION);   //Automatic screen rotation correction

        //UNCOMMENT THIS IF you're using a SECONDARY (facing toward screen) camera
        //or when you rotate the phone, sometimes the colors swap
        //rotation.setRotationInversion(true);

        //You can do this for certain phones which switch red and blue
        //It will rotate the display and detection by 180 degrees, making it upright
        rotation.setUnbiasedOrientation(ScreenOrientation.LANDSCAPE_WEST);

        //Set the beacon analysis method
        //Try them all and see what works!
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);
    }

    @Override
    public void loop() {
        super.loop();

        //Telemetry won't work here, but you can still do logging
    }

    @Override
    public Mat frame(Mat rgba, Mat gray) {
        //Run all extensions, then get matrices
        ColorBlobDetector detectorBlue = new ColorBlobDetector(Constants.COLOR_BLUE_LOWER, Constants.COLOR_BLUE_UPPER);
        ColorBlobDetector detectorRed = new ColorBlobDetector(Constants.COLOR_RED_LOWER, Constants.COLOR_RED_UPPER);
        detectorBlue.process(rgba);
        detectorRed.process(rgba);
        List<Contour> contoursBlue = detectorBlue.getContours();
        List<Contour> contoursRed = detectorRed.getContours();

        rgba = super.frame(rgba, gray);
        Imgproc.cvtColor(rgba, gray, Imgproc.COLOR_RGBA2GRAY);

        //Get beacon analysis
        Beacon.BeaconAnalysis beaconAnalysis = beacon.getAnalysis();

        //Display confidence
        Drawing.drawText(rgba, "Confidence: " + beaconAnalysis.getConfidenceString(),
                new Point(0, 50), 1.0f, new ColorGRAY(255));



        //Display beacon color
        Drawing.drawText(rgba, beaconAnalysis.getColorString(),
                new Point(0, 8), 1.0f, new ColorGRAY(255), Drawing.Anchor.BOTTOMLEFT);

        //Display FPS
        Drawing.drawText(rgba, "FPS: " + fps.getFPSString(), new Point(0, 24), 1.0f, new ColorRGBA("#ffffff")); //"#2196F3"

        //Display analysis method
        Drawing.drawText(rgba, beacon.getAnalysisMethod().toString() + " Analysis",
                new Point(width - 300, 40), 1.0f, new ColorRGBA("#FFC107"));

        //Display rotation sensor compensation
//        Drawing.drawText(rgba, "Rot: " + sensors.getScreenOrientationCompensation() + "("
//                + sensors.getActivityScreenOrientation().getAngle() + " act, "
//                + sensors.getScreenOrientation().getAngle() + " sen)", new Point(0, 50), 1.0f, new ColorRGBA("#ffffff"), Drawing.Anchor.BOTTOMLEFT); //"#2196F3"
        float average = 0;
        Log.d("CameraTest", "Average Reached.");
//        try {
        Log.d("CameraTest", "Try Entered.");
        Point blueCenter = beaconAnalysis.getBlueCenter(contoursRed, contoursBlue, rgba, gray);
            if (blueCenter != null) {
                blueCenterPoints.add((float) blueCenter.x);
                if (blueCenterPoints.size() > 10) {
                    blueCenterPoints.remove(0);
                    float adder = 0;
                    for (int i = 0; i < blueCenterPoints.size(); i++)
                        adder += blueCenterPoints.get(i);
                    average = adder / 10;
                }
            }
//        }
//        catch (Exception e) {
//            e.printStackTrace();
//        }
        Drawing.drawText(rgba, "Blue Center: "+Float.toString(average), new Point(0, 50), 1.0f, new ColorRGBA("#ffffff"), Drawing.Anchor.BOTTOMLEFT); //"#2196F3"
        return rgba;
    }
}