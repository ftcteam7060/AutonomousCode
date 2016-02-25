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

import android.text.format.Time;

import com.qualcomm.ftcrobotcontroller.autonomouslibs.PathSeg;
import com.qualcomm.ftcrobotcontroller.autonomouslibs.RobotStates;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.lasarobotics.vision.test.detection.ColorBlobDetector;
import org.lasarobotics.vision.test.detection.objects.Contour;
import org.lasarobotics.vision.test.ftc.resq.Beacon;
import org.lasarobotics.vision.test.image.Drawing;
import org.lasarobotics.vision.test.opmode.VisionOpMode;
import org.lasarobotics.vision.test.util.color.ColorHSV;
import org.lasarobotics.vision.test.util.color.ColorRGBA;
import org.opencv.core.Mat;
import org.opencv.core.Point;

import java.util.List;

/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */
public class RedClimb extends VisionOpMode {

    // The time in seconds to wait before starting.
    long mWaitTime = 0;
    //--------------------------------------------------------------------------
    // Robot Global Variables
    //--------------------------------------------------------------------------
  //  private GyroSensor          gyroSensor;
    private static RobotStates  robotState;
    private PathSeg[]           mCurrentPath;     // Array to hold current path`
    private int                 mCurrentSeg;      // Index of the current leg in the current path
    private int                 COUNTS_PER_INCH = 135; // Determined by trial and error measurements.
    private int                 mLeftEncoderTarget;
    private int                 mRightEncoderTarget;
    private long                mStartTime;
    //DcMotor debris_motor;
    Servo climberArm;
    DcMotorController   driveController;
    DcMotor             mRightMotor;
    DcMotor             mLeftMotor;
    DcMotor             tread_drive;

    // Loop cycle time stats variables
    public ElapsedTime  mRuntime = new ElapsedTime();   // Time into round.

    // Define driving paths as pairs of relative wheel movements in inches (left,right) plus speed %
    // Note: this is a dummy path, and is NOT likely to actually work with YOUR robot.
    final PathSeg[] mClimbPath = {
            new PathSeg( 58.0,    58.0,    0.3),
            new PathSeg( 12.0,    -12.0,   0.3),
            new PathSeg(-35.0, -35.0, 75.0, 0.5),
            new PathSeg(-100.0,   -100.0,   0.01),
    };
    @Override
    public void start() {
        super.start();
        // Setup Robot devices, set initial state and start game clock
        setDriveSpeed(0, 0, 0);        // Set target speed to zero
        runToPosition();            // Run to Position set by encoder targets
        mRuntime.reset();           // Zero game clock
        robotState = RobotStates.WAIT;

    }

    @Override
    public void init_loop()
    {
        // Keep resetting encoders and show the current values
        resetDriveEncoders();        // Reset Encoders to Zero
    }

    @Override
    public void init() {
        super.init();
        // Initialize motors
        //debris_motor = hardwareMap.dcMotor.get("debris_motor");
        driveController = hardwareMap.dcMotorController.get("drive_controller");
        mRightMotor = hardwareMap.dcMotor.get("right_drive");
        mLeftMotor = hardwareMap.dcMotor.get("left_drive");
        mRightMotor.setDirection(DcMotor.Direction.REVERSE);

        tread_drive = hardwareMap.dcMotor.get("tread_drive");
        climberArm = hardwareMap.servo.get("climber_arm");
        climberArm.setPosition(0.2);
        mStartTime = System.nanoTime();
    }

    @Override
    public void loop() {
        super.loop();
        telemetry.addData("RobotState:", robotState);
      //  telemetry.addData("Heading: ", gyroSensor.getHeading());
        stateMachine();
    }

    public void stateMachine() {
        //begins the state machine - states are defined in RobotStates enum class
        switch (robotState) {
            case WAIT:
                if (((System.nanoTime() - mStartTime) / 1000000000) > mWaitTime) {
                    robotState = RobotStates.START;
                }
                break;
            //begins the path toward the beacon
            case START:
                if(encodersAtZero()) { //event that happens to begin action
                    startPath(mClimbPath); //action to get to next state
                    robotState = RobotStates.TRAVERSE_TOWARD_BEACON;
                }
                break;
            //once you get to beacon, set the camera vision to active
            case TRAVERSE_TOWARD_BEACON:
                if (pathComplete()) {

                    robotState = RobotStates.STOP;
                }
                break;
            case STOP:
                stopMotors();
        }

    }


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
            setDriveSpeed(mCurrentPath[mCurrentSeg].mSpeed, mCurrentPath[mCurrentSeg].mSpeed, mCurrentPath[mCurrentSeg].mSpeed);

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
                mCurrentPath = null;
                mCurrentSeg = 0;
                setDriveSpeed(0, 0, 0);
                useConstantSpeed();
                return true;
            }
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
    void setDriveSpeed(double leftSpeed, double rightSpeed, double treadSpeed)
    {
        setDrivePower(leftSpeed, rightSpeed, treadSpeed);
    }

    //--------------------------------------------------------------------------
    // setDrivePower( LeftPower, RightPower);
    //--------------------------------------------------------------------------
    void setDrivePower(double leftPower, double rightPower, double treadPower)
    {
        mLeftMotor.setPower(Range.clip(leftPower, -1, 1));
        mRightMotor.setPower(Range.clip(rightPower, -1, 1));
        tread_drive.setPower(Range.clip(treadPower, -1, 1));
    }
    public void stopMotors()
    {
        mRightMotor.setPower(0.0);
        mLeftMotor.setPower(0.0);
    }
}
