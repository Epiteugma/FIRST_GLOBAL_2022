package com.z3db0y.davidlib;

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

        public Location(int pos, int side) {
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
        this.motors.forEach((location, motor) -> {
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
        }); 
    }

    public void driveRobotCentric(double forwardPower, double sidePower) {
        this.driveRobotCentric(forwardPower, sidePower, 0);
    }

    public Map<Location, Motor> getMotors() {
        return this.motors;
    }
}
