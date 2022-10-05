package com.z3db0y.davidlib;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.*;

public class DriveTrain {
    Type type;
    Map<Location, Motor> motors;

    public enum Type {
        TWOWHEELDRIVE,
        FOURWHEELDRIVE,
        SIXWHEELDRIVE
    }

    public enum Location {
        BACKLEFT,
        BACKRIGHT,
        CENTERLEFT,
        CENTERRIGHT,
        FRONTLEFT,
        FRONTRIGHT;

        public boolean isLeft() { return (this == BACKLEFT || this == CENTERLEFT || this == FRONTLEFT); }
        public boolean isRight() { return (this == BACKRIGHT || this == CENTERRIGHT || this == FRONTRIGHT); }
        public boolean isBack() { return (this == BACKLEFT || this == BACKRIGHT); }
        public boolean isCenter() { return (this == CENTERLEFT || this == CENTERRIGHT); }
        public boolean isFront() { return (this == FRONTLEFT || this == FRONTRIGHT); }
    }

    public DriveTrain(Type type, Map<Location, Motor> motors) {
        this.type = type;
        this.motors = motors;
    }


    public void driveRobotCentric(double forwardPower, double sidePower, double strafePower) {
        for(DriveTrain.Location location : this.motors.keySet()) {
            Motor motor = this.motors.get(location);
            double resultantPower = forwardPower;

            if(location.isLeft()) resultantPower -= sidePower;
            else resultantPower += sidePower;

            if(
                (location.isFront() && location.isLeft()) ||
                (location.isBack() && location.isRight())
            ) resultantPower += strafePower;

            if(
                (location.isFront() && location.isRight()) ||
                (location.isBack() && location.isLeft())
            ) resultantPower -= strafePower;

            motor.setPower(resultantPower);
        }
    }

    public boolean isBusy() {
        for(DriveTrain.Location location : this.motors.keySet()) {
            Motor motor = this.motors.get(location);
            if(motor.isBusy()) return true;
        }
        return false;
    }

    public void turn(double pow, double target, double wheelRadius, double ticksPerRev, double robotWidth) {
        double wheelCirc = 2 * Math.PI * wheelRadius;
        double cmPer90Deg = robotWidth * Math.PI / 2;
        double ticksPer90Deg = cmPer90Deg / wheelCirc * ticksPerRev;
        double targetTicks = ticksPer90Deg * target / 90;
        double leftTicks = 0;
        double rightTicks = 0;

        for(DriveTrain.Location location : this.motors.keySet()) {
            Motor motor = this.motors.get(location);
            if(location.isLeft()) leftTicks += motor.getCurrentPosition();
            else rightTicks += motor.getCurrentPosition();
        }
        double sideTickDelta = leftTicks - rightTicks;
        double rotation = sideTickDelta / ticksPer90Deg * 90;
        int direction = Math.abs(target - rotation) > 180 ? 1 : -1;

        for(DriveTrain.Location location : this.motors.keySet()) {
            Motor motor = this.motors.get(location);
            if(location.isLeft()) motor.setTargetPosition(motor.getCurrentPosition() - (int) (targetTicks * direction));
            else motor.setTargetPosition(motor.getCurrentPosition() + (int) (targetTicks * direction));
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(pow);
        }

        while(this.isBusy()) {
            for(Location l : this.motors.keySet()) {
                Logger.addDataDashboard(l.name() + " current", motors.get(l).getCurrentPosition());
                Logger.addDataDashboard(l.name() + " target", motors.get(l).getTargetPosition());
            }
            Logger.addDataDashboard("target", target);
            Logger.addDataDashboard("direction", direction);
            Logger.addDataDashboard("sideTickDelta", sideTickDelta);
            Logger.addDataDashboard("ticksPer90Deg", ticksPer90Deg);
            Logger.addDataDashboard("targetTicks", targetTicks);
            Logger.addDataDashboard("cmPer90Deg", cmPer90Deg);
            Logger.addDataDashboard("wheelCirc", wheelCirc);
            Logger.addDataDashboard("pow", pow);
            Logger.update();
        }

        for(DriveTrain.Location location : this.motors.keySet()) {
            Motor motor = this.motors.get(location);
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public Map<Location, Motor> getMotors() {
        return this.motors;
    }
}
