package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Configurable {
    // Tunable variables to tune from the dashboard, must be public and static so the dashboard can access them.
    // Always have to be in cm!!!
    // Constants and ratios
    public static double collectorPower = 1;
    public static double conveyorPower = 1;
    public static double shooterUpperPower = 0.5;
    public static double shooterLowerPower = 0.7;
    public static double shooterStep = 0.1;

    public static double shooterVeloStep = 200;

    public static double driveGearRatio = 14.0/2.0;

    // Wheel dimensions and distances
    // Old Wheels
    // public static double wheelRadius = 7/2;
    // New wheels
    public static double wheelRadius = 7.5;
    public static double wheelCircumference = 2 * Math.PI * wheelRadius;
    public static double centerToWheel = 21;


    public static double massOfTheBall = 40;
    public static double distanceFromTarget = 100;
    public static double gravity = 9.81;
    public static double angleToTarget = 45;
    public static double verticalDistanceToTarget = 250;


}
