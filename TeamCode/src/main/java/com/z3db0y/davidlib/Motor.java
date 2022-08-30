package com.z3db0y.davidlib;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

public class Motor extends DcMotorImplEx {
    boolean holdPosition = false;
    RunMode runMode;
    RunMode lastRunMode;

    public Motor(DcMotorController controller, int portNumber) {
        super(controller, portNumber);
    }

    public void runToPosition(int targetPosition, double power) {
        this.lastRunMode = this.runMode;
        while(Math.abs(targetPosition - this.getCurrentPosition()) > 0) {
            this.setMode(RunMode.RUN_USING_ENCODER);
            this.setPower(power);
        }
        this.setMode(this.lastRunMode);
        this.setPower(0);
    }

    public void runToPositionAsync(int targetPosition, double power) {
        this.setTargetPosition(targetPosition);
        super.setMode(RunMode.RUN_TO_POSITION);
        this.setPower(power);
    }

    public void setHoldPosition(boolean willHold) {
        this.holdPosition = willHold;
        holdPosition();
    }

    public boolean isHoldingPosition() {
        return this.holdPosition;
    }

    @Override
    public void setPower(double power) {
        this.setPowerInternal(power);
        this.holdPosition();
    }

    private void setPowerInternal(double power) {
        super.setPower(power);
    }

    private void holdPosition() {
        if(this.getPower() == 0) {
            this.lastRunMode = this.runMode;
            this.setTargetPosition(this.getCurrentPosition());
            this.setPowerInternal(1);
        }
    }

    @Override
    public void setVelocity(double velocity) {
        this.setVelocityInternal(velocity);
        this.holdPosition();
    }

    private void setVelocityInternal(double velocity) {
        super.setVelocity(velocity);
    }
}
