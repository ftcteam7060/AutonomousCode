package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import org.lasarobotics.vision.test.android.Camera;
import org.lasarobotics.vision.test.android.Cameras;
import org.opencv.core.Size;


/**
 * Created by heinz on 1/7/2016.
 */
public class BasicDriveTest extends LinearOpMode {
    DcMotorController driveController;
    DcMotor rightMotor;
    DcMotor leftMotor;
    DcMotor tread_drive;
    int desiredPosition = 0;
    final int TOLERANCE = 30;
    double drivePower = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors
        //debris_motor = hardwareMap.dcMotor.get("debris_motor");
        waitForNextHardwareCycle();
        driveController = hardwareMap.dcMotorController.get("drive_controller");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        this.initializeMotor(leftMotor);
        this.initializeMotor(rightMotor);

        // Wait for start button to be pressed.
        waitForStart();
        driveTo(inchesToEncoderCounts(32), inchesToEncoderCounts(32));
        driveTo(inchesToEncoderCounts(8), -inchesToEncoderCounts(8));
        driveTo(inchesToEncoderCounts(64), inchesToEncoderCounts(64));

    }

    private void driveTo(int leftEncoderDestination, int rightEncoderDestination) {
        leftEncoderDestination += leftMotor.getCurrentPosition();
        rightEncoderDestination += rightMotor.getCurrentPosition();

        while (     (rightMotor.getCurrentPosition() < (rightEncoderDestination - TOLERANCE)
                ||  rightMotor.getCurrentPosition() > (rightEncoderDestination + TOLERANCE))
                ||  (leftMotor.getCurrentPosition() < (leftEncoderDestination - TOLERANCE)
                ||  leftMotor.getCurrentPosition() > (leftEncoderDestination + TOLERANCE)))  {
            if (rightMotor.getCurrentPosition() < (rightEncoderDestination - TOLERANCE)) {
                rightMotor.setPower(drivePower);
            }
            else if (rightMotor.getCurrentPosition() > (rightEncoderDestination + TOLERANCE)) {
                rightMotor.setPower(-drivePower);
            }
            else {
                rightMotor.setPower(0);
            }
            if (leftMotor.getCurrentPosition() < (leftEncoderDestination - TOLERANCE)) {
                leftMotor.setPower(drivePower);
            }
            else if (leftMotor.getCurrentPosition() > (leftEncoderDestination + TOLERANCE)) {
                leftMotor.setPower(-drivePower);
            }
            else {
                leftMotor.setPower(0);
            }
            telemetry.addData("Left Motor", leftMotor.getCurrentPosition());
            telemetry.addData("Right Motor", rightMotor.getCurrentPosition());
        }
        rightMotor.setPower(0.0);
        leftMotor.setPower(0.0);
        telemetry.addData("Left Motor", leftMotor.getCurrentPosition());
        telemetry.addData("Right Motor", rightMotor.getCurrentPosition());
    }

    private void initializeMotor(DcMotor thisMotor) {
        thisMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        thisMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        while (thisMotor.isBusy()) {
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    /**
     * Converts inches to encoder ticks on First Team 7060's robot.
     * @param inches The distance in inches
     * @return encoderCounts
     */
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
}
