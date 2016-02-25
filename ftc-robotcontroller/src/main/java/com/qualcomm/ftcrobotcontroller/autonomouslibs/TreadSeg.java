package com.qualcomm.ftcrobotcontroller.autonomouslibs;

/**
 * Define a "PathSegment" object, used for building a path for the robot to follow.
 */
public class TreadSeg
{
    public double mLeft;
    public double mRight;
    public double mTread;
    public double mSpeed;

    // Constructor
    public TreadSeg(double Left, double Right, double Tread, double Speed)
    {
        mLeft = Left;
        mRight = Right;
        mTread = Tread;
        mSpeed = Speed;
    }
}