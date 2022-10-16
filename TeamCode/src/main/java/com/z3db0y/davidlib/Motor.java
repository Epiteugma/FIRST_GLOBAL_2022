package com.z3db0y.davidlib;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;

public class Motor extends DcMotorImplEx {
    boolean hold = false;
    boolean inHoldState = false;
    double power = 0;
    double lastPowerUpdate;
    RunMode runMode = RunMode.RUN_WITHOUT_ENCODER;

    public Motor(DcMotorImplEx motor) {
        super(motor.getController(), motor.getPortNumber());
    }

    public void holdPosition() {
        System.out.println("Hold: " + hold);
        System.out.println("power: " + power);
        System.out.println("inHoldState: " + inHoldState);
        if(hold && this.power == 0) {
            if(!inHoldState) {
                System.out.println("holdPos true");
                super.setTargetPosition(super.getCurrentPosition());
                super.setMode(RunMode.RUN_TO_POSITION);
                super.setPower(1);
                inHoldState = true;
            }
        } else {
            inHoldState = false;
            super.setMode(this.runMode);
            super.setPower(power);
        }
    }

    public void setHoldPosition(boolean hold) {
        this.hold = hold;
        holdPosition();
    }

    public void setPower(double power) {
        this.lastPowerUpdate = System.currentTimeMillis();
        this.power = power;
        holdPosition();
    }

    public void setMode(RunMode runMode) {
        this.runMode = runMode;
        holdPosition();
    }

    public boolean atTargetPosition(){
        return this.getCurrentPosition() == this.getTargetPosition();
    }

    public RunMode getMode() {
        return this.runMode;
    }

    public double getPower() {
        return this.power;
    }

    public boolean isStalled() {
        return (super.getVelocity() < (super.getMotorType().getAchieveableMaxTicksPerSecond() * super.getPower() * 0.5)) && power != 0 && this.lastPowerUpdate + 250 < System.currentTimeMillis();
    }
}
