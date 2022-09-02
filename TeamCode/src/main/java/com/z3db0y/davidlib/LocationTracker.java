package com.z3db0y.davidlib;

import java.util.*;

public class LocationTracker {
    boolean isInitialized = false;
    DriveTrain driveTrain;
    int motorTicksPerRevolution;
    double robotWidth;
    double wheelRadius;
    Vector currentLocation = new Vector(0, 0, 0);
    int lastTickAverage = 0;

    public class Parameters {
        public DriveTrain driveTrain;
        public int motorTicksPerRevolution;
        public double robotWidth;
        public double wheelRadius;
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
    }

    public Vector updatePosition() {
        if(!this.isInitialized) return null;

        // TODO: check location tracking
        List<Motor> leftMotors = new ArrayList<>();
        List<Motor> rightMotors = new ArrayList<>();
        
        int leftTickSum = 0;
        int rightTickSum = 0;
        int tickSum = 0;

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

            tickSum += motor.getCurrentPosition();
        }

        int leftTickAverage = (int) (leftTickSum / leftMotors.size());
        int rightTickAverage = (int) (rightTickSum / rightMotors.size());

        int sideTickDelta = leftTickAverage - rightTickAverage;
        double ticksPerDegree = this.robotWidth / 360 / this.wheelRadius / ((leftMotors.size() + rightMotors.size()) / 2);

        double robotRotation = sideTickDelta / ticksPerDegree;
        while(robotRotation > 360) robotRotation -= 360;

        double rotation180 = robotRotation;
        if(rotation180 > 180) rotation180 = 360 - rotation180;

        double rotation180Shifted = Math.abs(robotRotation - 90);
        if(rotation180Shifted > 180) rotation180Shifted = 360 - rotation180Shifted;

        double xMultiplier = 1 - rotation180/90;
        double zMultiplier = 1 - rotation180Shifted/90;

        int tickAverage = tickSum / (leftMotors.size() + rightMotors.size());
        int ticksMoved = tickAverage;
        if(lastTickAverage != 0) {
            ticksMoved -= lastTickAverage;
        }

        int revolutionsMoved = ticksMoved / this.motorTicksPerRevolution;
        double distanceMoved = revolutionsMoved * (2 * Math.PI * this.wheelRadius);

        this.currentLocation.X += distanceMoved * xMultiplier;
        this.currentLocation.Z += distanceMoved * zMultiplier;

        lastTickAverage = tickAverage;
        return this.currentLocation;
    }

    public Vector getPosition() {
        return this.currentLocation;
    }
}
