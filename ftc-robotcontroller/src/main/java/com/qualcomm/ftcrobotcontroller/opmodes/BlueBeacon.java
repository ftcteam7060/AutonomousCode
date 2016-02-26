/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftcrobotcontroller.autonomouslibs.RobotStates;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.ftcrobotcontroller.autonomouslibs.*;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.lasarobotics.vision.detection.ColorBlobDetector;
import org.lasarobotics.vision.detection.objects.Contour;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.image.Drawing;
import org.lasarobotics.vision.opmode.VisionOpMode;
import org.lasarobotics.vision.util.color.ColorHSV;
import org.lasarobotics.vision.util.color.ColorRGBA;
import org.opencv.core.Mat;
import org.opencv.core.Point;

import java.util.List;

/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */
public class BlueBeacon extends VisionOpMode {

    //--------------------------------------------------------------------------
    // Vision Global Variables
    //--------------------------------------------------------------------------
    private static final ColorHSV   lowerBoundRed = new ColorHSV((int) (305 / 360.0 * 255.0), (int) (0.200 * 255.0), (int) (0.300 * 255.0));
    private static final ColorHSV   upperBoundRed = new ColorHSV((int) ((360.0 + 5.0) / 360.0 * 255.0), 255, 255);
    private static final ColorHSV   lowerBoundBlue = new ColorHSV((int) (170.0 / 360.0 * 255.0), (int) (0.200 * 255.0), (int) (0.750 * 255.0));
    private static final ColorHSV   upperBoundBlue = new ColorHSV((int) (227.0 / 360.0 * 255.0), 255, 255);
    private Beacon.BeaconAnalysis   colorAnalysis = new Beacon.BeaconAnalysis();
    private ColorBlobDetector       detectorRed;
    private ColorBlobDetector       detectorBlue;
    private boolean                 noError = true;
    private boolean                 visionIsActive = true;
    private double                  beaconWidth = 21.8;
    private double                  beaconHeight = 14.5;
    public double                   beaconCenterX;
    public double                   beaconCenterY;
    public String                   beaconColor = new String();
    public String                   analyConfi = new String();
    private int                     widthFrame = 910;
    private int                     heightFrame = 864;
    private Beacon                  beacon;
    //--------------------------------------------------------------------------
    // Robot Global Variables
    //--------------------------------------------------------------------------
    private GyroSensor          gyroSensor;
    private static RobotStates  robotState;
    private PathSeg[]           mCurrentPath;     // Array to hold current path`
    private int                 mCurrentSeg;      // Index of the current leg in the current path
    private int                 COUNTS_PER_INCH = 135; // Determined by trial and error measurements.
    private int                 mLeftEncoderTarget;
    private int                 mRightEncoderTarget;
    private int                 mGyroTarget;
    private int                 mInitialHeading;
    private boolean             turningLeft = false;
    private boolean             turningRight = false;
    //DcMotor debris_motor;
    Servo climberArm;
    DcMotorController   driveController;
    DcMotor             mRightMotor;
    DcMotor             mLeftMotor;
    DcMotor             tread_drive;
/*    DcMotor             firstWinch;
    DcMotor             secondWinch; */


    // Loop cycle time stats variables
    public ElapsedTime  mRuntime = new ElapsedTime();   // Time into round.
    private ElapsedTime mStateTime = new ElapsedTime();  // Time into current state

    // Define driving paths as pairs of relative wheel movements in inches (left,right) plus speed %
    // Note: this is a dummy path, and is NOT likely to actually work with YOUR robot.
    final PathSeg[] mBeaconPath_1 = {
            new PathSeg(  1.0,    1.0,    0.3),
//            new PathSeg(  0.0,    11.0,   0.3),
//            new PathSeg( 80.0,    80.0,   0.4),
//            new PathSeg(  0.0,     0.0,   0.0),
//            new PathSeg(-12.0,    12.0,   0.3),
//            new PathSeg( 16.0,    16.0,   0.3),
//            centers on beacon
//            new PathSeg(-34.0,   -34.0,   0.3),  // Changed from 28 to 34 inches
//            new PathSeg( 13.0,     0.0,   0.3),
//            new PathSeg( 20.0,    20.0,   0.3)
    };
    final PathSeg[] mInitialTurn = {
            new PathSeg(-2.0, 2.0, 0.3)
    };

    final PathSeg[] mBeaconPath = {
            new PathSeg(  1.0,    1.0,    0.3), //initial movement to get away from wall
            new PathSeg( 11.0,    0.0,   0.3), //initial turn toward beacon
            new PathSeg( 83.0,    83.0,   0.4), //straight-line distance to beacon
            new PathSeg(  0.0,     0.0,   0.0), //pause
            new PathSeg( 10.0,    -10.0,   0.3), //final turn to center on beacon
            new PathSeg( 28.0,    28.0,   0.3), //final movement to beacon (before depositing climbers
            //centers on beacon
            //new PathSeg(-34.0,   -34.0,   0.3),  // Changed from 28 to 34 inches
            //new PathSeg( 13.0,     0.0,   0.3),
            //new PathSeg( 20.0,    20.0,   0.3)
    };
    final PathSeg[] mBeaconPath_2 = {
//            new PathSeg(  0.0,    11.0,   0.3),
            new PathSeg( 80.0,    80.0,   0.4),
            new PathSeg(  0.0,     0.0,   0.0),
            new PathSeg(-12.0,    12.0,   0.3),
            new PathSeg( 16.0,    16.0,   0.3),
            new PathSeg(-34.0,   -34.0,   0.3),  // Changed from 28 to 34 inches
            new PathSeg( 13.0,     0.0,   0.3),
            new PathSeg( 20.0,    20.0,   0.3)
    };
    //path to deposit climbers
    final PathSeg[] mClimberDeposit = {
            new PathSeg(  2.5,     2.5,   0.2),
            new PathSeg(  0.0,     2.0,   0.3),
            new PathSeg(  0.0,     0.0,   0.0)
    };
    boolean begunSearching = false;
    final PathSeg[] mSearchForBeacon = {
            new PathSeg(  5.0,    -5.0, 0.3),
            new PathSeg(  0.0,     0.0, 0.0),
            new PathSeg(-10.0,    10.0, 0.3)

    };

    final PathSeg[] mParking = {
            new PathSeg(0.0, 0.0, 0.0),
            new PathSeg (-11.0, 11.0, 0.3),
            new PathSeg (10.0, 10.0, 0.3)
    };

    @Override
    public void start() {
        super.start();
        // Setup Robot devices, set initial state and start game clock
        setDriveSpeed(0, 0);        // Set target speed to zero
        runToPosition();            // Run to Position set by encoder targets
        mRuntime.reset();           // Zero game clock
        robotState = RobotStates.START;
    }

    @Override
    public void init_loop()
    {
        // Keep resetting encoders and show the current values
        resetDriveEncoders();        // Reset Encoders to Zero
        telemetry.addData("ENC", String.format("L:R %d:%d", getLeftPosition(), getRightPosition()));
    }

    @Override
    public void init() {
        super.init();

        detectorRed = new ColorBlobDetector(lowerBoundRed, upperBoundRed);
        detectorBlue = new ColorBlobDetector(lowerBoundBlue, upperBoundBlue);

        //Initialize Gyro
        //    gyroSensor = hardwareMap.gyroSensor.get("gyro_sensor");

        // Initialize motors
        //debris_motor = hardwareMap.dcMotor.get("debris_motor");
        driveController = hardwareMap.dcMotorController.get("drive_controller");
        mRightMotor = hardwareMap.dcMotor.get("right_drive");
        mLeftMotor = hardwareMap.dcMotor.get("left_drive");
        mRightMotor.setDirection(DcMotor.Direction.REVERSE);

        tread_drive = hardwareMap.dcMotor.get("tread_drive");
//        linear_slide = hardwareMap.dcMotor.get("linear_slide");
//        vertical_pivot = hardwareMap.dcMotor.get("vertical_pivot");
//        gyroSensor = hardwareMap.gyroSensor.get("gyro_sensor");
//        firstWinch = hardwareMap.dcMotor.get("first_winch");
//        secondWinch = hardwareMap.dcMotor.get("second_winch");
        climberArm = hardwareMap.servo.get("climber_arm");
        climberArm.setPosition(0.4);
    }

    @Override
    public void loop() {
        super.loop();
        telemetry.addData("RobotState:", robotState);
        //  telemetry.addData("Heading: ", gyroSensor.getHeading());
//        experimentalStateMachine();
        stateMachine();
    }
    public void experimentalStateMachine() {
        //begins the state machine - states are defined in RobotStates enum class
        switch (robotState) {
            //begins the path toward the beacon
            case START:
                if (encodersAtZero()) { //event that happens to begin action
                    startPath(mInitialTurn); //action to get to next state
                    robotState = RobotStates.INIT_TURN_COMPLETE;
                }
                break;
            //once you get to beacon, set the camera vision to active
            case INIT_TURN_COMPLETE:
                mInitialHeading = 315;
                if (headingReached(mInitialHeading)) {

                    // robotState = RobotStates.TRAVERSE_TOWARD_BEACON;
                }
                //displays information
                else {
                    // Display Diagnostic data for this state.
                    telemetry.addData("1", String.format("%d of %d. L %5d:%5s - R %5d:%5d ",
                            mCurrentSeg, mCurrentPath.length,
                            mLeftEncoderTarget, getLeftPosition(),
                            mRightEncoderTarget, getRightPosition()));
                }
                break;
        }
    }
    public void stateMachine() {
        //begins the state machine - states are defined in RobotStates enum class
        switch (robotState) {
            //begins the path toward the beacon
            case START:
                if(encodersAtZero()) { //event that happens to begin action
                    startPath(mBeaconPath); //action to get to next state
                    robotState = RobotStates.TRAVERSE_TOWARD_BEACON;
                }
                break;
            //once you get to beacon, set the camera vision to active
            case TRAVERSE_TOWARD_BEACON:
                if (pathComplete()) {

                    robotState = RobotStates.DEPOSIT_CLIMBERS;
                }
                //displays information
                else {
                    // Display Diagnostic data for this state.
                    telemetry.addData("1", String.format("%d of %d. L %5d:%5s - R %5d:%5d ",
                            mCurrentSeg, mCurrentPath.length,
                            mLeftEncoderTarget, getLeftPosition(),
                            mRightEncoderTarget, getRightPosition()));
                }
                break;
            //search for the beacon
            case LOOKING_FOR_BEACON:
                double BEACON_TARGET = 150;
                double BEACON_MARGIN = 25;
                telemetry.addData("beaconCenterY", beaconCenterY);
                telemetry.addData("beaconCenterX", beaconCenterX);

                if (colorAnalysis.isBeaconFound()) {
                    robotState = RobotStates.BEACON_FOUND;
                }
                //if found, go to next state
                if (Math.abs(beaconCenterX - BEACON_TARGET) < BEACON_MARGIN) {
                    telemetry.addData("Beacon", "Found");
                    //    telemetry.addData("Beacon Size", beacon.getSize());

                    mLeftMotor.setPower(0);
                    runToPosition();
                    robotState = RobotStates.BEACON_FOUND;
                }
//                 If the beaconCenter is to the Right of the center of the phone, turn right.
                else if (beaconCenterX > BEACON_TARGET) {
                    // Turn Right
                    turningLeft = false;
                    telemetry.addData("Beacon", "Right");
                    if (!turningRight) {
                        startPath(new PathSeg[]{
                                new PathSeg(3, -3, 0.2)
                        });
                        turningRight = true;
                    }
                }
                else if (beaconCenterX < BEACON_TARGET && beaconCenterX != 0) {
                    // Turn Left
                    telemetry.addData("Beacon", "Left");
                    turningRight = false;
                    if (!turningLeft) {
                        startPath(new PathSeg[]{
                                new PathSeg(-3, 3, 0.2)
                        });
                        turningLeft = true;
                    }
                }
                else if (!begunSearching) {
                    // Scan for Beacon
                    startPath(mSearchForBeacon);
                    begunSearching = true;
                    telemetry.addData("Beacon", "Searching");
                    turningLeft = false;
                    turningRight = false;
                }
                else {
                    telemetry.addData("Beacon", "Searching");

                }

                break;
            //state that begins process to dumping climbers
            case BEACON_FOUND:
//                telemetry.addData("Beacon Found, Confidence: ", colorAnalysis.getConfidenceString());
//                telemetry.addData("beaconCenterX", beaconCenterX);
//                telemetry.addData("(widthFrame/2) + 30", (widthFrame / 2) + 30);
//                telemetry.addData("beaconCenterY", beaconCenterY);
                // telemetry.addData("ButtonCenter", beacon.);
                //if (beaconCenterX <= ((widthFrame/2) + 30) && beaconCenterX >= ((widthFrame/2) - 30)) {
                //start path to get in`to ideal position to dump climbers
                if(encodersAtZero()) {
                    startPath(mClimberDeposit);
                    robotState = RobotStates.ADJUST_TO_CLIMBERS;
                }
                else {
                    resetDriveEncoders();
                    // telemetry.addData("ENC", mLeftMotor.getCurrentPosition(), mRightMotor.getCurrentPosition());
                }
                break;
            //checks to make sure path is complete and robot is in position to dump climbers
            case ADJUST_TO_CLIMBERS:
                if(pathComplete()) {
                    robotState = RobotStates.DEPOSIT_CLIMBERS;
                }
                break;
            //dump climbers into the bin
            case DEPOSIT_CLIMBERS:
                if (climberArm.getPosition() == 0.2) {
                    climberArm.setPosition(0.8);
                    // climberArm.setPosition(0.4);
                    robotState = RobotStates.STOP;
                }
                else {
                    climberArm.setPosition(0.2);
                }
                break;
            case PARKING:
                if (pathComplete()&& encodersAtZero()){
                    startPath(mParking);
                    robotState = robotState.TRANSITION_TO_STOP;
                }
                else {
                    resetDriveEncoders();
                }
                break;
            case TRANSITION_TO_STOP:
                if(pathComplete()) {
                    robotState = RobotStates.STOP;
                }
                break;
            case STOP:
                stopMotors();
                //climberArm.setPosition(0.2);
                break;
        }

    }

    private int inchesToEncoderCounts(int inches) {
        // Wheel and encoder variables
        final int WHEEL_DIAMETER = 4; //measured in inches
        final double WHEEL_CIRCUM = WHEEL_DIAMETER * 3.14159;
        double wheelRotations = inches / WHEEL_CIRCUM;
        final int GEAR_RATIO = 1; //could be 40
        double motorRotations = wheelRotations * GEAR_RATIO;
        // 1440 represents pulses per revolution, but 1120 is 78%, so 1120
        int encoderCounts = (int) (motorRotations * 1440);

        //Magic calculations
        encoderCounts *= 30;
        encoderCounts /= 26;
        return encoderCounts;
    }

//    public void encoderForward(int counts) {
//        mRightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
//        mLeftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
//        //Reverse the left drive so that positive values correlate with forward movement.
//        mLeftMotor.setDirection(DcMotor.Direction.REVERSE);
//        //Calibrate Gy
//        gyroSensor.calibrate();
//        while (gyroSensor.isCalibrating()) {
//            pause(50);
//        }
//        mRightMotor.setTargetPosition(this.inchesToEncoderCounts(counts));
//        mLeftMotor.setTargetPosition(this.inchesToEncoderCounts(counts));
//        driveController.setMotorChannelMode(1, DcMotorController.RunMode.RUN_TO_POSITION);
//        driveController.setMotorChannelMode(2, DcMotorController.RunMode.RUN_TO_POSITION);
//        mRightMotor.setPower(0.3);
//        mLeftMotor.setPower(0.3);
//        telemetry.addData("CurrentMotorPosition: ", driveController.getMotorCurrentPosition(1));
//        telemetry.addData("TargetMotorPosition: ", driveController.getMotorTargetPosition(1));
//        //If we're within 5 encoder counts of our destination, turn off motors.
//        if (Math.abs(driveController.getMotorCurrentPosition(1) - driveController.getMotorTargetPosition(1)) < 5) {
//            mRightMotor.setPower(0);
//            mLeftMotor.setPower(0);
//            robotState = RobotStates.TURN_TO_BEACON;
//        }
//    }
//
//    /**
//     * This program is designed to be run repeatedly in the main State Machine loop until the
//     * desired heading is reached.
//     *
//     * @param degrees
//     * @return
//     */
//    public void turnLeft(int degrees) {
//
//        gyroSensor.calibrate();
//        // telemetry.addData("Rotation ", gyroSensor.getRotation());
//        while (gyroSensor.isCalibrating()) {
//            pause(50);
//        }
//
//        final int HEADING_RANGE = 5;
//        driveController.setMotorChannelMode(1, DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
//        driveController.setMotorChannelMode(2, DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
//
//        int currentHeading = gyroSensor.getHeading();
//        telemetry.addData("02", currentHeading);
//
//        if (gyroSensor.getHeading() > 0 && gyroSensor.getHeading() < 180) {
//            int desiredHeading1 = degrees - gyroSensor.getHeading();
//            int desiredHeading2 = 360 - desiredHeading1;
//            while (gyroSensor.getHeading() - desiredHeading2 > 2) {
//                mRightMotor.setPower(0.5);
//                mLeftMotor.setPower(-0.5);
//                telemetry.addData("Gyro Heading", gyroSensor.getHeading());
//            }
//        } else {
//            int desiredHeading = gyroSensor.getHeading() - degrees;
//            while (gyroSensor.getHeading() - desiredHeading > 2) {
//                mRightMotor.setPower(0.5);
//                mLeftMotor.setPower(-0.5);
//                telemetry.addData("Gyro Heading", gyroSensor.getHeading());
//            }
//        }
//    }

//******************************************************************************
//******************************************************************************
//******************************************************************************
// Motor Execution Code
//******************************************************************************
//******************************************************************************
//******************************************************************************

    //--------------------------------------------------------------------------
    // syncEncoders()
    // Load the current encoder values into the Target Values
    // Essentially sync's the software with the hardware
    //--------------------------------------------------------------------------
    void syncEncoders()
    {
        //	get and set the encoder targets
        mLeftEncoderTarget = mLeftMotor.getCurrentPosition();
        mRightEncoderTarget = mRightMotor.getCurrentPosition();
    }
    //--------------------------------------------------------------------------
    // runToPosition ()
    // Set both drive motors to encoder servo mode (requires encoders)
    //--------------------------------------------------------------------------
    public void runToPosition()
    {
        setDriveMode(DcMotorController.RunMode.RUN_TO_POSITION);
    }
    //--------------------------------------------------------------------------
    // encodersAtZero()
    // Return true if both encoders read zero (or close)
    //--------------------------------------------------------------------------
    boolean encodersAtZero()
    {
        return ((Math.abs(getLeftPosition()) < 5) && (Math.abs(getRightPosition()) < 5));
    }
    //--------------------------------------------------------------------------
    // getLeftPosition ()
    // Return Left Encoder count
    //--------------------------------------------------------------------------
    int getLeftPosition()
    {
        return mLeftMotor.getCurrentPosition();
    }

    int getGyroHeading()
    {
        return gyroSensor.getHeading();
    }

    //--------------------------------------------------------------------------
    // getRightPosition ()
    // Return Right Encoder count
    //--------------------------------------------------------------------------
    int getRightPosition()
    {
        return mRightMotor.getCurrentPosition();
    }
    //--------------------------------------------------------------------------
    // setDriveMode ()
    // Set both drive motors to new mode if they need changing.
    //--------------------------------------------------------------------------
    public void setDriveMode(DcMotorController.RunMode mode)
    {
        // Ensure the motors are in the correct mode.
        if (mLeftMotor.getChannelMode() != mode)
            mLeftMotor.setChannelMode(mode);

        if (mRightMotor.getChannelMode() != mode)
            mRightMotor.setChannelMode(mode);
    }
    //--------------------------------------------------------------------------
    // resetDriveEncoders()
    // Reset both drive motor encoders, and clear current encoder targets.
    //--------------------------------------------------------------------------
    public void resetDriveEncoders()
    {
        setEncoderTarget(0, 0);
        setDriveMode(DcMotorController.RunMode.RESET_ENCODERS);
    }
    //--------------------------------------------------------------------------
    // setEncoderTarget( LeftEncoder, RightEncoder);
    // Sets Absolute Encoder Position
    //--------------------------------------------------------------------------
    void setEncoderTarget(int leftEncoder, int rightEncoder)
    {
        mLeftMotor.setTargetPosition(mLeftEncoderTarget = leftEncoder);
        mRightMotor.setTargetPosition(mRightEncoderTarget = rightEncoder);
    }

    void setGyroHeading(int heading)
    {
        mGyroTarget = heading;
    }
    /*
            Begin the first leg of the path array that is passed in.
            Calls startSeg() to actually load the encoder targets.
         */
    private void startPath(PathSeg[] path)
    {
        mCurrentPath = path;    // Initialize path array
        mCurrentSeg = 0;
        syncEncoders();        // Lock in the current position
        runToPosition();        // Enable RunToPosition mode
        startSeg();             // Execute the current (first) Leg
    }

    /*
        Starts the current leg of the current path.
        Must call startPath() once before calling this
        Each leg adds the new relative movement onto the running encoder totals.
        By not reading and using the actual encoder values, this avoids accumulating errors.
        Increments the leg number after loading the current encoder targets
     */
    private void startSeg()
    {
        int Left;
        int Right;

        if (mCurrentPath != null)
        {
            // Load up the next motion based on the current segemnt.
            Left  = (int)(mCurrentPath[mCurrentSeg].mLeft * COUNTS_PER_INCH);
            Right = (int)(mCurrentPath[mCurrentSeg].mRight * COUNTS_PER_INCH);
            addEncoderTarget(Left, Right);
            setDriveSpeed(mCurrentPath[mCurrentSeg].mSpeed, mCurrentPath[mCurrentSeg].mSpeed);

            mCurrentSeg++;  // Move index to next segment of path
        }
    }
    /*
        Determines if the current path is complete
        As each segment completes, the next segment is started unless there are no more.
        Returns true if the last leg has completed and the robot is stopped.
     */
    private boolean pathComplete()
    {
        // Wait for this Segement to end and then see what's next.
        if (moveComplete())
        {
            // Start next Segement if there is one.
            if (mCurrentSeg < mCurrentPath.length)
            {
                startSeg();
            }
            else  // Otherwise, stop and return done
            {
//                mCurrentPath = null;
                mCurrentSeg = 0;
                setDriveSpeed(0, 0);
                useConstantSpeed();
                return true;
            }
        }
        return false;
    }

    private boolean headingReached(int desiredHeading)
    {
        // Wait for this Segement to end and then see what's next.
        if (turnComplete(desiredHeading))
        {
            // Start next Segement if there is one.
//            if (mCurrentSeg < mCurrentPath.length)
//            {
//                startSeg();
//            }
//            else  // Otherwise, stop and return done
//            {
            mCurrentPath = null;
            mCurrentSeg = 0;
            setDriveSpeed(0, 0);
            useConstantSpeed();
            return true;
//            }
        }
        return false;
    }
    //--------------------------------------------------------------------------
    // moveComplete()
    // Return true if motors have both reached the desired encoder target
    //--------------------------------------------------------------------------
    boolean moveComplete()
    {
        //  return (!mLeftMotor.isBusy() && !mRightMotor.isBusy());
        return ((Math.abs(getLeftPosition() - mLeftEncoderTarget) < 10) &&
                (Math.abs(getRightPosition() - mRightEncoderTarget) < 10));
    }

    boolean turnComplete(int gyroHeading)
    {
        //  return (!mLeftMotor.isBusy() && !mRightMotor.isBusy());
        return ((Math.abs(getGyroHeading() - gyroHeading) < 10));
    }
    //--------------------------------------------------------------------------
    // useConstantSpeed ()
    // Set both drive motors to constant speed (requires encoders)
    //--------------------------------------------------------------------------
    public void useConstantSpeed()
    {
        setDriveMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }
    //--------------------------------------------------------------------------
    // addEncoderTarget( LeftEncoder, RightEncoder);
    // Sets relative Encoder Position.  Offset current targets with passed data
    //--------------------------------------------------------------------------
    void addEncoderTarget(int leftEncoder, int rightEncoder)
    {
        mLeftMotor.setTargetPosition(mLeftEncoderTarget += leftEncoder);
        mRightMotor.setTargetPosition(mRightEncoderTarget += rightEncoder);
    }

    //--------------------------------------------------------------------------
    // setDriveSpeed( LeftSpeed, RightSpeed);
    //--------------------------------------------------------------------------
    void setDriveSpeed(double leftSpeed, double rightSpeed)
    {
        setDrivePower(leftSpeed, rightSpeed);
    }

    //--------------------------------------------------------------------------
    // setDrivePower( LeftPower, RightPower);
    //--------------------------------------------------------------------------
    void setDrivePower(double leftPower, double rightPower)
    {
        mLeftMotor.setPower(Range.clip(leftPower, -1, 1));
        mRightMotor.setPower(Range.clip(rightPower, -1, 1));
    }
//        int desiredHeading = degrees;
//
//        mRightMotor.setPower(0.2);
//        mLeftMotor.setPower(-0.2);
//        currentHeading = gyroSensor.getHeading();
//        telemetry.addData("03", "Current Heading:" + new Integer(currentHeading).toString());
//        telemetry.addData("04", "Desired Heading:" + new Integer(desiredHeading).toString());
//        telemetry.addData("04", "Loop Completed.");
//        gyroSensor.
//
//        if (Math.abs(desiredHeading - currentHeading) < HEADING_RANGE) {
//            return true;
//        }
//        else {
//            return false;
//        }


//    private void turnRobot(int desiredHeading) {
//
//       // int currentHeading = gyroSensor.getHeading();
//        driveController.setMotorChannelMode(1, DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
//        driveController.setMotorChannelMode(2, DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
//        double turnPower = 0.15;
//        int margin = 2;
//
//        while (Math.abs(currentHeading - desiredHeading) > margin) {
//            telemetry.addData("TurnMode", "Turning");
//            // If either the currentHeading or the desiredHeading are greater than 180, but if they
//            // are not BOTH greater than 180...
////            if ((currentHeading > 180 || desiredHeading > 180) &&
////                    !(currentHeading > 180 && desiredHeading > 180)) {
////                if (Math.abs(currentHeading - desiredHeading) > 180) {
////                    if (currentHeading < desiredHeading) {
////                        // Turn Left
////                        mRightMotor.setPower(turnPower);
////                        mLeftMotor.setPower(-turnPower);
////
////                    } else {
////                        // Turn Right
////                        mRightMotor.setPower(-turnPower);
////                        mLeftMotor.setPower(turnPower);
////
////                    }
////                } else {
////                    if (currentHeading > desiredHeading) {
////                        // Turn Left
////                        mRightMotor.setPower(turnPower);
////                        mLeftMotor.setPower(-turnPower);
////
////                    } else {
////                        // Turn Right
////                        mRightMotor.setPower(turnPower);
////                        mLeftMotor.setPower(-turnPower);
////
////                    }
////                }
////            } else {
////                if (currentHeading > desiredHeading) {
////                    // Turn Left
////                    mRightMotor.setPower(turnPower);
////                    mLeftMotor.setPower(turnPower);
////                } else {
////                    // Turn Right
////                }
////            }
//        }
//        telemetry.addData("TurnMode", "Completed");
//    }

    public void stopMotors()
    {
        mRightMotor.setPower(0.0);
        mLeftMotor.setPower(0.0);
    }

    @Override
    public Mat frame(Mat rgba, Mat gray) {
        if (visionIsActive) {
            try {
                //Process the frame for the color blobs
                detectorRed.process(rgba);
                detectorBlue.process(rgba);

                //Get the list of contours
                List<Contour> contoursRed = detectorRed.getContours();
                List<Contour> contoursBlue = detectorBlue.getContours();

                //Get color analysis
                beacon = new Beacon();
                colorAnalysis = beacon.analyzeFrame(contoursRed, contoursBlue, rgba, gray); //beacon.analyzeFrame(contoursRed, contoursBlue, rgba, gray, ScreenOrientation.LANDSCAPE);

                beaconCenterX = colorAnalysis.getCenter().x; //getBeaconCenter().x;
                beaconCenterY = colorAnalysis.getCenter().y;// getBeaconCenter().y;
                beaconColor = colorAnalysis.toString();
                analyConfi = colorAnalysis.getConfidenceString();

                noError = true;
            } catch (Exception e) {
                Drawing.drawText(rgba, "Analysis Error", new Point(0, 8), 1.0f, new ColorRGBA("#F44336"), Drawing.Anchor.BOTTOMLEFT);
                noError = false;
                e.printStackTrace();
            }
        }

        return rgba;
    }
    private void pause(int duration)
    {
        try {
            Thread.sleep(duration);
        }
        catch (Exception e)
        {
            //Gotta catch em all!
            telemetry.addData("Error", " in turn.");
        }
    }
}
