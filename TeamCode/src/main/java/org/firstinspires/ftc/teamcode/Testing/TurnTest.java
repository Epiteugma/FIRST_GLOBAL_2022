package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.z3db0y.davidlib.DriveTrain;
import com.z3db0y.davidlib.Logger;
import com.z3db0y.davidlib.Motor;

import org.firstinspires.ftc.teamcode.Configurable;

import java.util.Arrays;
import java.util.HashMap;

@TeleOp(name = "TurnTest", group = "FGC22")
public class TurnTest extends LinearOpMode {

    Motor leftSide;
    Motor rightSide;
    DriveTrain driveTrain;



    @Override
    public void runOpMode() {
        leftSide = new Motor(hardwareMap.get(DcMotorImplEx.class, "leftSide"));
        rightSide = new Motor(hardwareMap.get(DcMotorImplEx.class, "rightSide"));
        HashMap<DriveTrain.Location, Motor> driveTrainMotors = new HashMap<>();
        driveTrainMotors.put(DriveTrain.Location.FRONTLEFT, leftSide);
        driveTrainMotors.put(DriveTrain.Location.FRONTRIGHT, rightSide);
        driveTrain = new DriveTrain(DriveTrain.Type.TWOWHEELDRIVE, driveTrainMotors);


        rightSide.setDirection(DcMotor.Direction.REVERSE);
        Arrays.asList(leftSide, rightSide).forEach(motor -> {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            motor.setHoldPosition(true);
        });

        waitForStart();

        driveTrain.turn(1.0, 45.0, Configurable.wheelRadius, Configurable.motorTicksPerRevolution, Configurable.robotWidth);
        while (opModeIsActive()) {
            leftSide.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x);
            rightSide.setPower(gamepad1.left_stick_y + gamepad1.right_stick_x);
        }
    }
}
