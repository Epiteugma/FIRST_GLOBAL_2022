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
        BACKLEFT(0, 1),
        BACKRIGHT(0, 0),
        CENTERLEFT(1, 1),
        CENTERRIGHT(1, 0),
        FRONTLEFT(2, 1),
        FRONTRIGHT(2, 0);

        int pos;
        int side;

        Location(int pos, int side) {
            this.pos = pos;
            this.side = side;
        }

        public boolean isLeft() { return (this.side == 1); }
        public boolean isRight() { return (this.side == 0); }
        public boolean isBack() { return (this.pos == 0); }
        public boolean isCenter() { return (this.pos == 1); }
        public boolean isFront() { return (this.pos == 2); }
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

    public void turn(double pow, double target) {
        double wheelCirc = 2 * Math.PI * 4.5;
        double cmPer90Deg = 23.9 * Math.PI / 2;
        double ticksPer90Deg = cmPer90Deg / wheelCirc * 560;
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
