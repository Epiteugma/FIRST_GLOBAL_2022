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


    public static double heightOfTarget = 250;
    public static double horizontalDistanceToTarget = 200;
    public static double groundToShooterCenter = 36;
    public static double verticalDistanceToTarget = heightOfTarget - groundToShooterCenter;

    public static double shooterAngleOnRobot = 43.0;

    public static double shooterWheelRadius = 2.65;
    public static double shooterGearRatio = 90.0/30.0;

    public static double maxRads = 11;
    public static double minRads = 7.5;
    public static double radsAdditionRange = maxRads - minRads;
    public static double shooterStep = 0.25;
    public static double shooterMarginOfError = 0.85;
    public static double shooterTicksPerRev = 1120 * shooterGearRatio;

}
