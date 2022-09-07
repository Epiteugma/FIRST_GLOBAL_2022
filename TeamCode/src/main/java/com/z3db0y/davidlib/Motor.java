package com.z3db0y.davidlib;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Configurable;

@Config
public class Motor {
    boolean holdPosition = false;
    DcMotorEx motor;
    DcMotorEx.RunMode runMode;
    DcMotorEx.RunMode lastRunMode;
    double power = 0;

    public Motor(HardwareMap hardwareMap, String motorName) {
        this.motor = hardwareMap.get(DcMotorEx.class, motorName);
    }

    public void runToPosition(int targetPosition, double power) {
        this.lastRunMode = this.runMode;
        while(Math.abs(targetPosition - this.getCurrentPosition()) > 0) {
            this.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            this.setPower(power);
        }
        this.setMode(this.lastRunMode);
        this.setPower(0);
    }

    public void runToPositionAsync(int targetPosition, double power) {
        this.setTargetPosition(targetPosition);
        this.motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        this.setPower(power);
    }

    public void setHoldPosition(boolean willHold) {
        this.holdPosition = willHold;
        holdPosition();
    }

    public boolean isHoldingPosition() {
        return this.holdPosition;
    }

    public void setPower(double power) {
        this.power = power;
        holdPosition();
        if(!this.holdPosition) this.setPowerInternal(power);
    }

    private void setPowerInternal(double power) {
        this.motor.setPower(power);
    }

    public double getPower() {
        return this.power;
    }

    public void setTargetPosition(int ticks) {
        this.motor.setTargetPosition(ticks);
    }

    public int getCurrentPosition() {
        return this.motor.getCurrentPosition();
    }

    public void setMode(DcMotorEx.RunMode runMode) {
        this.motor.setMode(runMode);
    }

    public void resetEncoder(){
        this.motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(this.runMode);
    }

    public void setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior behavior) {
        this.motor.setZeroPowerBehavior(behavior);
    }

    public DcMotorEx.ZeroPowerBehavior getZeroPowerBehavior() { return this.motor.getZeroPowerBehavior(); }

    private void holdPosition() {
        if(this.getPower() == 0) {
            this.lastRunMode = this.runMode;
            this.setTargetPosition(this.getCurrentPosition());
            this.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            this.setPowerInternal(1);
        } else this.setMode(this.lastRunMode);
    }

    public double getVelocity() {
        return this.motor.getVelocity();
    }

    public void setVelocity(double angularRate, AngleUnit angleUnit) {
        this.motor.setVelocity(angularRate, angleUnit);
    }

    public double getVelocity(AngleUnit angleUnit) {
        return this.motor.getVelocity(angleUnit);
    }

    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        this.motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f));
    }

    public void setDirection(DcMotorEx.Direction direction) {
        this.motor.setDirection(direction);
    }

    public boolean isBusy() {
        return this.motor.isBusy();
    }

    public int getTargetPosition() { return this.motor.getTargetPosition(); }
}
