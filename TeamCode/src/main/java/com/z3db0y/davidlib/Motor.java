package com.z3db0y.davidlib;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;

public class Motor extends DcMotorImplEx {
    boolean hold = false;
    double power = 0;

    public Motor(DcMotorImplEx motor) {
        super(motor.getController(), motor.getPortNumber());
    }

    public void holdPosition() {
        if(hold && this.getPower() == 0) {
            this.setTargetPosition(this.getCurrentPosition());
            this.setMode(RunMode.RUN_TO_POSITION);
            super.setPower(1);
        }
    }

    public void setHoldPosition(boolean hold) {
        this.hold = hold;
        holdPosition();
    }

    public void setPower(double power) {
        this.power = power;
        super.setPower(power);
        holdPosition();
    }

    public double getPower() {
        return power;
    }
}
