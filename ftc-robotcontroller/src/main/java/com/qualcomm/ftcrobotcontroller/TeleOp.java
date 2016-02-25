package com.qualcomm.ftcrobotcontroller;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by heinz on 11/20/2015.
 */
public class TeleOp extends OpMode {

    // This is the previous value of the gamepad 1's x button.
    boolean prevGamepad1X = false;
    boolean toggleFront = false;
    //DcMotor debris_motor;
    Servo climberArm;
    DcMotor right_drive;
    DcMotor left_drive;
    DcMotor tread_drive;
    /* Enable Winch
    DcMotor firstWinch;
    DcMotor secondWinch;*/
    private double scale_leftYJoy;
    private double scale_rightYJoy;
    //This Logical switch turns on once when a button is pressed, then returns to false.
    private boolean onX1Press;
    //    DcMotor linear_slide;
//    DcMotor vertical_pivot;
    //GyroSensor gyroSensor;

    /**
     * Velocity Scaling from 0 to 1, sets the velocity scale of the drive motors.
     */
//    double VELOCITY = 0.5;
    public TeleOp(){

    }

    public void init() {
        //debris_motor = hardwareMap.dcMotor.get("debris_motor");
        /* Enable Winch
        firstWinch = hardwareMap.dcMotor.get("first_winch");
        secondWinch = hardwareMap.dcMotor.get("second_winch"); */
        climberArm = hardwareMap.servo.get("climber_arm");
        right_drive = hardwareMap.dcMotor.get("right_drive");
        left_drive = hardwareMap.dcMotor.get("left_drive");
        tread_drive = hardwareMap.dcMotor.get("tread_drive");
//        linear_slide = hardwareMap.dcMotor.get("linear_slide");
//        vertical_pivot = hardwareMap.dcMotor.get("vertical_pivot");
      //  gyroSensor = hardwareMap.gyroSensor.get("gyro_sensor");
    }

    public void start(){
        right_drive.setDirection(DcMotor.Direction.REVERSE);
    }

    public void loop(){
        float leftYJoy = -gamepad1.left_stick_y;
        float rightYJoy = -gamepad1.right_stick_y;
        float leftYJoy_2 = -gamepad2.left_stick_y;
        float rightYJoy_2 = -gamepad2.right_stick_y;
        scale_leftYJoy = scale_motor_power(leftYJoy);
        scale_rightYJoy = scale_motor_power(rightYJoy);

        if (!toggleFront) {
            right_drive.setPower(scale_rightYJoy);
            left_drive.setPower(scale_leftYJoy);
        }
        else {
            right_drive.setPower(-scale_leftYJoy);
            left_drive.setPower(-scale_rightYJoy);
        }
        telemetry.addData("01", right_drive.getCurrentPosition());
//        vertical_pivot.setPower(scale_leftYJoy_2);
        // Use left bumper and right bumper to move treads
        if (gamepad1.left_bumper == true)
        {
            tread_drive.setPower(1.0);
        }
        else if (gamepad1.right_bumper == true)
        {
            tread_drive.setPower(-1.0);
        }
        else
        {
            tread_drive.setPower(0.0);
        }
        if (gamepad1.x == true && prevGamepad1X == false) {
            toggleFront = !toggleFront;
        }
        prevGamepad1X = gamepad1.x;
        //Conditional structure for moving linear slide
        /* Enable Winch
        if (gamepad2.right_bumper == true)
        {
            firstWinch.setPower(1.0);
            secondWinch.setPower(1.0);
        }
        else if (gamepad2.left_bumper == true)
        {
            firstWinch.setPower(-1.0);
            secondWinch.setPower(-1.0);
        }
        else
        {
            firstWinch.setPower(0.0);
            secondWinch.setPower(0.0);
        }
        */
        //Conditional structure for moving arm pivot
        if (gamepad2.x == true)
        {
            toggleFront = !toggleFront;
//            vertical_pivot.setPower(0.4);
        }
        else if (gamepad2.y == true)
        {
//            vertical_pivot.setPower(-0.4);
        }
        else
        {
//            vertical_pivot.setPower(0.0);
        }

        //climber arm movement
        if(gamepad2.a == true) {
            climberArm.setPosition(0.8);
        }

        else {
            climberArm.setPosition(0.4);
        }
//        if (gamepad2.left_bumper == true)
//        {
//            linear_slide.setPower(1.0);
//        }
//        else if (gamepad2.right_bumper == true)
//         {
//            linear_slide.setPower(-1.0);
//        }
//        else
//        {
//            linear_slide.setPower(0.0);
//        }
//        else
//        {
//            tread_drive.setPower(0.0);
//        }
//        if (gamepad1.left_bumper == true && gamepad1.right_bumper == true)
//        {
//            right_motor.setPower(1.0);
//            left_motor.setPower(1.0);
//        }
//        else if (gamepad1.left_bumper == true)
//        {
//            left_motor.setPower(1.0);
//        }
//        else if (gamepad1.right_bumper == true)
//        {
//            right_motor.setPower(1.0);
//        }
//        else
//        {
//            right_motor.setPower(0.0);
//            left_motor.setPower(0.0);
//        }
        telemetry.addData("01", scale_leftYJoy);
    }

    double scale_motor_power (double p_power)
    {
        //
        // Assume no scaling.
        //
        double l_scale = 0.0f;
//        p_power *= VELOCITY;
        //
        // Ensure the values are legal.
        //
        double l_power = Range.clip(p_power, -1, 1);

        double[] l_array =
                { 0.00, 0.05, 0.09, 0.10, 0.12
                        , 0.15, 0.18, 0.24, 0.30, 0.36
                        , 0.43, 0.50, 0.60, 0.72, 0.85
                        , 1.00, 1.00
                };

        //
        // Get the corresponding index for the specified argument/parameter.
        //
        int l_index = (int) (l_power * 16.0);
        if (l_index < 0)
        {
            l_index = -l_index;
        }
        else if (l_index > 16)
        {
            l_index = 16;
        }

        if (l_power < 0)
        {
            l_scale = -l_array[l_index];
        }
        else
        {
            l_scale = l_array[l_index];
        }

        return l_scale;

    } // PushBotManual::scale_motor_power
}
