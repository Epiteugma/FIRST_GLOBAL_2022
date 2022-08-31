package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Configurable {
    // Tunable variables to tune from the dashboard, must be public and static so the dashboard can access them.
    // Always have to be in cm!!!
    // Constants and ratios
    public static double collectorPower = 1;
    public static double conveyorPower = 1;

    // Wheel dimensions and distances
    // Old Wheels
    // public static double wheelRadius = 7/2;
    // New wheels
//    public static double wheelRadius = 7.5;
//    public static double wheelCircumference = 2 * Math.PI * wheelRadius;
//    public static double centerToWheel = 21;
//    public static double driveGearRatio = 14.0/2.0;


    public static double massOfTheBall = 40;
    public static double horizontalDistanceToTarget = 260;
    public static double gravity = 9.81;
    public static double angleToTarget = 65.0;
    public static double heightOfTarget = 250;
    public static double groundToShooterCenter = 28;
    public static double verticalDistanceToTarget = heightOfTarget - groundToShooterCenter;
    public static double shooterWheelRadius = 2.25;
    public static double shooterStep = 0.1;
    public static double shooterVeloRadStep = 2;
    public static double shooterGearRatio = 135.0/30.0;
    public static double shooterTicksPerRev = 1120 * shooterGearRatio;

}
