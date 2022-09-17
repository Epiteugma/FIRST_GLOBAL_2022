package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.z3db0y.davidlib.DriveTrain;
import com.z3db0y.davidlib.LocationTracker;
import com.z3db0y.davidlib.Motor;
import com.z3db0y.davidlib.Vector;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.*;

public class DriveNew extends LinearOpMode {
    Map<String, Motor> motors = new HashMap<>();
    List<Toggles> toggles = Arrays.asList(Toggles.COLLECTOR);
    DriveTrain driveTrain;
    LocationTracker tracker;
    Vector location;
    Vector sinkCenterLocation = Configurable.sinkCenterLocation;
    BNO055IMU imu;

    enum Toggles {
        SHOOTER,
        COLLECTOR;

        public double prevTime = 0;
    }

    void initMotors() {
        motors.put("leftSide", new Motor(hardwareMap.get(DcMotorImplEx.class, "leftSide")));
        motors.put("rightSide", new Motor(hardwareMap.get(DcMotorImplEx.class, "rightSide")));
        motors.put("shooterUp", new Motor(hardwareMap.get(DcMotorImplEx.class, "shooterUp")));
        motors.put("shooterDown", new Motor(hardwareMap.get(DcMotorImplEx.class, "shooterDown")));
        motors.put("conveyor", new Motor(hardwareMap.get(DcMotorImplEx.class, "conveyor")));
        motors.put("collector", new Motor(hardwareMap.get(DcMotorImplEx.class, "collector")));

        motors.forEach((name, motor) -> {
            motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(RunMode.RUN_USING_ENCODER);
        });

        Map<DriveTrain.Location, Motor> driveTrainMotors = new HashMap<>();
        driveTrainMotors.put(DriveTrain.Location.BACKLEFT, motors.get("leftSide"));
        driveTrainMotors.put(DriveTrain.Location.BACKRIGHT, motors.get("rightSide"));
        driveTrain = new DriveTrain(DriveTrain.Type.TWOWHEELDRIVE, driveTrainMotors);
    }

    void initLocator() {
        tracker = new LocationTracker();
        location = new Vector(0, 0, 0);

        LocationTracker.Parameters params = new LocationTracker.Parameters();
        params.wheelRadius = Configurable.wheelRadius;
        params.motorTicksPerRevolution = Configurable.motorTicksPerRevolution;
        params.robotWidth = Configurable.robotWidth;
        params.driveTrain = driveTrain;

        tracker.initialize(params);
    }

    void initIMU() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
    }

    void initSelf() {
        this.initMotors();
        this.initLocator();
        this.initIMU();
    }

    void track() {
        location = tracker.updatePosition(imu.getAngularOrientation().firstAngle);
    }

    void drive(double left_stick_y, double right_stick_x) {
        driveTrain.driveRobotCentric(-left_stick_y, right_stick_x, 0);
    }

    void toggles() {
        class Toggler {
            public void toggle(Toggles toggle) {
                toggle.prevTime = System.currentTimeMillis();
                if(toggles.contains(toggle)) toggles.remove(toggle);
                else toggles.add(toggle);
            }

            public boolean check(Object cond, Toggles toggle) {
                boolean pass = false;
                if(cond instanceof Number) pass = ((Number) cond).doubleValue() > 0;
                else if(cond instanceof Boolean) pass = (boolean) cond;

                return pass && toggle.prevTime + 0.5 < System.currentTimeMillis();
            }
        }

        Toggler toggler = new Toggler();
        if(toggler.check(gamepad1.a, Toggles.SHOOTER)) {
            toggler.toggle(Toggles.SHOOTER);
        }
    }

    void shooter() {
        PIDFCoefficients shooterUpPID = Configurable.shooterUpCoeffs;
        PIDFCoefficients shooterDownPID = Configurable.shooterDownCoeffs;
        double velocity = this.calculateShooterVelocity();

        motors.get("shooterUp").setPIDFCoefficients(RunMode.RUN_USING_ENCODER, shooterUpPID);
        motors.get("shooterDown").setPIDFCoefficients(RunMode.RUN_USING_ENCODER, shooterDownPID);

        if(toggles.contains(Toggles.SHOOTER)) {
            motors.get("shooterUp").setVelocity(velocity, AngleUnit.DEGREES);
            motors.get("shooterDown").setVelocity(velocity, AngleUnit.DEGREES);
        } else {
            motors.get("shooterUp").setPower(0);
            motors.get("shooterDown").setPower(0);
        }
    }

    double calculateShooterVelocity() {
        return 0;
    }

    void collector() {
        if(toggles.contains(Toggles.COLLECTOR)) {
            motors.get("collector").setPower(1);
        } else {
            motors.get("collector").setPower(0);
        }
    }

    @Override
    public void runOpMode() {
        initSelf();
        waitForStart();
        while (opModeIsActive()) {
            track();
            drive(gamepad1.left_stick_y, gamepad1.right_stick_x);
            shooter();
            collector();

            toggles();
        }
    }
}
