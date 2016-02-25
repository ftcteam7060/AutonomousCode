package com.qualcomm.ftcrobotcontroller.autonomouslibs;

import android.graphics.Path;

/**
 * Define a "PathSegment" object, used for building a path for the robot to follow.
 */
public class PathSeg
{
    public double mLeft;
    public double mRight;
    public double mSpeed;
    public double mTread;
    // Constructor
    public PathSeg(double Left, double Right, double Speed)
    {
        mLeft = Left; //Distance left wheel should travel (negative means traveling backward)
        mRight = Right; //Distance right wheel should travel (negative means traveling backward)
        mSpeed = Speed; //Speed at which both wheels should operate
    }
    public PathSeg() {
        mLeft = 0;
        mRight = 0;
        mSpeed = 0;
    }
    public PathSeg(double Left, double Right, double Tread, double Speed){
        mLeft = Left;
        mRight = Right; //Distance right wheel should travel (negative means traveling backward)
        mSpeed = Speed; //Speed at which both wheels should operate
        mTread = Tread;
    }
}