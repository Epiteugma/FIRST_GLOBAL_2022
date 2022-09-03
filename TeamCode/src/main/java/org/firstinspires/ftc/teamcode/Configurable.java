package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class Configurable {
    // Tunable variables to tune from the dashboard, must be public and static so the dashboard can access them.
    // Always have to be in cm!!!
    // Constants and ratios
    public static double collectorPower = 1;
    public static double conveyorPower = 1;

    // Wheel dimensions and distances
    // Old Wheels
     public static double wheelRadius = 90.0/10.0/2.0;
    // New wheels
//    public static double wheelRadius = 7.5;
//    public static double wheelCircumference = 2 * Math.PI * wheelRadius;
//    public static double centerToWheel = 21;
//    public static double driveGearRatio = 14.0/2.0;


    public static PIDFCoefficients shooterUpCoeffs = new PIDFCoefficients(0.3, 0.05, 0.3, 3); // PID coefficients that need to be tuned probably through FTC dashboard
    public static PIDFCoefficients shooterDownCoeffs = new PIDFCoefficients(0.3, 0.05, 0.3, 3); // PID coefficients that need to be tuned probably through FTC dashboard

    public static double heightOfTarget = 250;
    public static double horizontalDistanceToTarget = 260;
    public static double groundToShooterCenter = 40;
    public static double verticalDistanceToTarget = heightOfTarget - groundToShooterCenter;

    public static double shooterWheelRadiusStart = 2.5;
    public static double shooterWheelMaxExpansion = 0.5;
    public static double shooterGearRatio = 125.0/30.0;

    public static double maxShooterVeloRads = 11.5;
    public static double minShooterVeloRads = 7.5;
    public static double radsAdditionRange = maxShooterVeloRads - minShooterVeloRads;
    public static double shooterStep = 0.25;
    public static double shooterMarginOfError = 0.85;

}
