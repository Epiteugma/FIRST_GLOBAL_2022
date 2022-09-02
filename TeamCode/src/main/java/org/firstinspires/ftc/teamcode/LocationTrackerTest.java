package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.z3db0y.davidlib.DriveTrain;
import com.z3db0y.davidlib.LocationTracker;
import com.z3db0y.davidlib.Logger;
import com.z3db0y.davidlib.Motor;
import com.z3db0y.davidlib.Vector;

import java.util.Arrays;
import java.util.HashMap;

//! IMPORTANT NOTE:
//! This code requires a robot's parameters
//! to work properly. Currently, the parameters
//! are calibrated based on the test robot.
@TeleOp(name = "Location Tracker Test", group = "FGC22")
public class LocationTrackerTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu.initialize(imuParameters);

        Motor leftMotor = new Motor(hardwareMap, "leftSide");
        Motor rightMotor = new Motor(hardwareMap, "rightSide");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        HashMap<DriveTrain.Location, Motor> motors = new HashMap<>();
        motors.put(DriveTrain.Location.BACKLEFT, leftMotor);
        motors.put(DriveTrain.Location.BACKRIGHT, rightMotor);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Arrays.asList(leftMotor, rightMotor).forEach(motor -> { motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); });

        DriveTrain driveTrain = new DriveTrain(DriveTrain.Type.TWOWHEELDRIVE, motors);
        LocationTracker tracker = new LocationTracker();
        LocationTracker.Parameters params = new LocationTracker.Parameters();
        params.driveTrain = driveTrain;
        params.motorTicksPerRevolution = 28 * 20;
        params.robotWidth = 23.9;
        params.wheelRadius = 4.5;
        //* params.telemetry = telemetry;
        tracker.initialize(params);

        Vector target = new Vector(30, 0, 30);
        waitForStart();

        double targetAngle = tracker.getPosition().anglesTo(target).get(0);
        double currentAngle;
        if (targetAngle > 180) targetAngle -= 360;

        double turnThreshold = 0.5;
        double rangeMin = targetAngle + turnThreshold / 2;
        double rangeMax = targetAngle - turnThreshold / 2;

        double diff;
        do {
            currentAngle = imu.getAngularOrientation().firstAngle;
            if (currentAngle == 180 && targetAngle < 0) currentAngle = -180;

            double target360 = targetAngle < 0 ? targetAngle + 360 : targetAngle;
            double current360 = currentAngle < 0 ? currentAngle + 360 : currentAngle;
            double diff360 = current360 - target360;
            int directionMultiplier = Math.abs(diff360) > 180 ? (diff360 > 0 ? 1 : -1) : (diff360 > 0 ? -1 : 1);

            leftMotor.setPower(0.5 * directionMultiplier);
            rightMotor.setPower(-0.5 * directionMultiplier);
            Logger.update();
        } while (currentAngle < rangeMin || currentAngle > rangeMax);
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        while(Math.abs(tracker.getPosition().X - target.X) > 3 && Math.abs(tracker.getPosition().Z - target.Z) > 3) {
            tracker.updatePosition();
            leftMotor.setPower(0.5);
            rightMotor.setPower(0.5);
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

}
