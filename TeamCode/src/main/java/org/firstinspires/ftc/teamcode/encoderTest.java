package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Arrays;

@TeleOp(name = "encoderTest", group = "FGC22")

public class encoderTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotor leftMotor = hardwareMap.get(DcMotor.class, "leftSide");
        DcMotor rightMotor = hardwareMap.get(DcMotor.class, "rightSide");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        Arrays.asList(leftMotor, rightMotor).forEach(motor -> { motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); });
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Left Encoder: ", leftMotor.getCurrentPosition());
            telemetry.addData("Right Encoder: ", rightMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
