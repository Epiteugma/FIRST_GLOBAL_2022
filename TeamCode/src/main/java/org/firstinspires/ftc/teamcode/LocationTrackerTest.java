package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
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
@Config
@TeleOp(name = "Location Tracker Test", group = "FGC22")
public class LocationTrackerTest extends LinearOpMode {

    double integralSum = 0;
    double lastError = 0;
    ElapsedTime timer = new ElapsedTime();
    public static PIDCoefficients pidCoeffs = new PIDCoefficients(0.1, 0.1, 0.1); //tune these

    public double angleWrap(double radians) {
        while(radians > Math.PI) radians -= 2 * Math.PI;
        while(radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }

    public double pidControl(double reference, double state){
        double error = angleWrap(reference - state);
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;
        timer.reset();
        return (error * LocationTrackerTest.pidCoeffs.p) + (integralSum * LocationTrackerTest.pidCoeffs.i) + (derivative * LocationTrackerTest.pidCoeffs.d);
    }

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
        while(Math.abs(tracker.getPosition().X - target.X) > 3 || Math.abs(tracker.getPosition().Z - target.Z) > 3) {
            tracker.updatePosition();
            double currentAngle = imu.getAngularOrientation().firstAngle;
            if(currentAngle < 0) currentAngle += 360;
            currentAngle = 360 - currentAngle;

            double power = pidControl(Math.toRadians(targetAngle), Math.toRadians(currentAngle));
            leftMotor.setPower(0.1 + power);
            rightMotor.setPower(0.1 - power);
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

}
