package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.z3db0y.davidlib.Logger;
import com.z3db0y.davidlib.Motor;

import java.util.Arrays;

@TeleOp(name = "TurnTest", group = "FGC22")
public class TurnTest extends LinearOpMode {

    Motor leftSide;
    Motor rightSide;

    public void turn(double pow, double target) {
        double wheelCirc = 2 * Math.PI * 4.5;
        double cmPer90Deg = 23.9 * Math.PI / 2;
        double ticksPer90Deg = cmPer90Deg / wheelCirc * 560;
        double targetTicks = ticksPer90Deg * target / 90;
        double sideTickDelta = leftSide.getCurrentPosition() - rightSide.getCurrentPosition();
        double rotation = sideTickDelta / ticksPer90Deg * 90;
        int direction = Math.abs(target - rotation) > 180 ? 1 : -1;

        double targetLeft = leftSide.getCurrentPosition() + (targetTicks * direction);
        double targetRight = rightSide.getCurrentPosition() - (targetTicks * direction);

        leftSide.setTargetPosition((int) targetLeft);
        rightSide.setTargetPosition((int) targetRight);

        leftSide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSide.setPower(pow);
        rightSide.setPower(pow);

        while(Math.abs(leftSide.getCurrentPosition() - leftSide.getTargetPosition()) > 3 && Math.abs(leftSide.getCurrentPosition() - leftSide.getTargetPosition()) > 3) {
            Logger.addDataDashboard("targetLeft", targetLeft);
            Logger.addDataDashboard("targetRight", targetRight);
            Logger.addDataDashboard("leftSide.getCurrentPosition()", leftSide.getCurrentPosition());
            Logger.addDataDashboard("rightSide.getCurrentPosition()", rightSide.getCurrentPosition());
            Logger.addDataDashboard("rotation", rotation);
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

        leftSide.setPower(0);
        rightSide.setPower(0);
        leftSide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void runOpMode() {
        leftSide = new Motor(hardwareMap, "leftSide");
        rightSide = new Motor(hardwareMap, "rightSide");

        Arrays.asList(leftSide, rightSide).forEach(motor -> {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        });

        rightSide.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();

        turn(1, 45);
        double start = System.currentTimeMillis();
        while (System.currentTimeMillis() < start + 1000) {}
    }
}
