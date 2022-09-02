package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "encoderTest", group = "FGC22")

public class encoderTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Left Encoder: ", hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "leftSide").getCurrentPosition());
            telemetry.addData("Right Encoder: ", hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "rightSide").getCurrentPosition());
            telemetry.update();
        }
    }
}
