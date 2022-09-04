package com.z3db0y.davidlib;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.*;

public class LocationTracker {
    boolean isInitialized = false;
    DriveTrain driveTrain;
    int motorTicksPerRevolution;
    double robotWidth;
    double wheelRadius;
    Vector currentLocation = new Vector(0, 0, 0);
    Telemetry telemetry;
    int lastLeftTickSum = 0;
    int lastRightTickSum = 0;
    double lastRobotAngle = 0;

    public static class Parameters {
        public DriveTrain driveTrain;
        public int motorTicksPerRevolution;
        public double robotWidth;
        public double wheelRadius;
        public Telemetry telemetry;
    }

    public void initialize(Parameters params) {
        if(params.driveTrain == null) return;
        if(params.motorTicksPerRevolution == 0) return;
        if(params.robotWidth == 0) return;

        this.isInitialized = true;
        this.driveTrain = params.driveTrain;
        this.motorTicksPerRevolution = params.motorTicksPerRevolution;
        this.robotWidth = params.robotWidth;
        this.wheelRadius = params.wheelRadius;
        this.telemetry = params.telemetry;
    }


    PIDCoefficients turnCoeffs = new PIDCoefficients(0.1, 0.1, 0.1); //tune these
    PIDCoefficients driveCoeffs = new PIDCoefficients(0.1, 0.1, 0.1); //tune these

    public Vector updatePosition() {
        if(!this.isInitialized) return null;

        List<Motor> leftMotors = new ArrayList<>();
        List<Motor> rightMotors = new ArrayList<>();

        int leftTickSum = 0;
        int rightTickSum = 0;

        for(DriveTrain.Location location : this.driveTrain.getMotors().keySet()) {
            Motor motor = this.driveTrain.getMotors().get(location);
            if(location.isLeft()) {
                leftTickSum += motor.getCurrentPosition();
                leftMotors.add(motor);
            }
            else {
                rightTickSum += motor.getCurrentPosition();
                rightMotors.add(motor);
            }
        }

        int tickSum = (leftTickSum + rightTickSum) - (lastLeftTickSum + lastRightTickSum);

        int leftTickAverage = (leftTickSum - lastLeftTickSum) / leftMotors.size();
        int rightTickAverage = (rightTickSum - lastRightTickSum) / rightMotors.size();
        int tickAverage = tickSum / (leftMotors.size() + rightMotors.size());

        int sideTickDelta = rightTickAverage - leftTickAverage;
        double distancePer90Degrees = 2 * Math.PI * this.robotWidth / 4;
        double cmPerRevolution = 2 * Math.PI * this.wheelRadius;
        double ticksPerCM = this.motorTicksPerRevolution / cmPerRevolution;
        double revolutionsPerDegree = distancePer90Degrees / cmPerRevolution / 90;
        double ticksPerDegree = this.motorTicksPerRevolution * revolutionsPerDegree;

        double robotAngle = lastRobotAngle - sideTickDelta / ticksPerDegree;
        while (robotAngle < 0) robotAngle += 360;
        while (robotAngle > 360) robotAngle -= 360;

        double rotation180 = robotAngle;
        if(rotation180 > 180) rotation180 = 360 - rotation180;

        double rotation180Shifted = Math.abs(robotAngle - 90);
        if(rotation180Shifted > 180) rotation180Shifted = 360 - rotation180Shifted;

        double xMultiplier = 1 - rotation180/90;
        double zMultiplier = 1 - rotation180Shifted/90;

        this.currentLocation.X += tickAverage / ticksPerCM * xMultiplier;
        this.currentLocation.Z += tickAverage / ticksPerCM * zMultiplier;

        if(this.telemetry != null) {
            this.telemetry.addData("X", this.currentLocation.X);
            this.telemetry.addData("Z", this.currentLocation.Z);
            this.telemetry.addData("X Multiplier", xMultiplier);
            this.telemetry.addData("Z Multiplier", zMultiplier);
            this.telemetry.addData("Left Tick Average", leftTickAverage);
            this.telemetry.addData("Right Tick Average", rightTickAverage);
            this.telemetry.addData("Tick Average", tickAverage);
            this.telemetry.addData("Robot Angle", robotAngle);
            this.telemetry.update();
        }

        lastLeftTickSum = leftTickSum;
        lastRightTickSum = rightTickSum;
        lastRobotAngle = robotAngle;
        return this.currentLocation;
    }

    //hello

    public Vector getPosition() {
        return this.currentLocation;
    }
}
