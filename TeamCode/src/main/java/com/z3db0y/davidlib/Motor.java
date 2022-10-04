package com.z3db0y.davidlib;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;

public class Motor extends DcMotorImplEx {
    boolean hold = false;
    double power = 0;
    RunMode runMode = RunMode.RUN_WITHOUT_ENCODER;

    public Motor(DcMotorImplEx motor) {
        super(motor.getController(), motor.getPortNumber());
    }

    public void holdPosition() {
        if(hold && this.getPower() == 0) {
            System.out.println("holdPos true");
            this.setTargetPosition(this.getCurrentPosition());
            super.setMode(RunMode.RUN_TO_POSITION);
            super.setPower(1);
        }
    }

    public void setHoldPosition(boolean hold) {
        this.hold = hold;
        holdPosition();
    }

    public void setPower(double power) {
        this.power = power;
        super.setMode(this.runMode);
        super.setPower(power);
        holdPosition();
    }

    public void setMode(RunMode runMode) {
        this.runMode = runMode;
        super.setMode(runMode);
        holdPosition();
    }

    public RunMode getMode() {
        return this.runMode;
    }

    public double getPower() {
        return power;
    }
}
