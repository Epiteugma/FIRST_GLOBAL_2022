package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.z3db0y.davidlib.Logger;

import java.util.Arrays;
@Disabled
@TeleOp(name = "encoderTest", group = "FGC22")

public class encoderTest extends LinearOpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftMotor = hardwareMap.get(DcMotor.class, "leftSide");
        rightMotor = hardwareMap.get(DcMotor.class, "rightSide");
        Arrays.asList(leftMotor, rightMotor).forEach(motor -> {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        });
        waitForStart();

        while (opModeIsActive()) {
            driveTrainControl();
            telemetry.addData("Left Encoder: ", leftMotor.getCurrentPosition());
            telemetry.addData("Right Encoder: ", rightMotor.getCurrentPosition());
            telemetry.update();
        }
    }

    private void driveTrainControl(){
        double yleft = gamepad1.left_stick_y * -1; //inverted
        double yright = gamepad1.right_stick_y * -1; //inverted
        Logger.addDataDashboard("Left Stick Y", yleft);
        Logger.addDataDashboard("Right Stick Y", yright);
        leftMotor.setPower(yleft);
        rightMotor.setPower(yright);
    }
}
