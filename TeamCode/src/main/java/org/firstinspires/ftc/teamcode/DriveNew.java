package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.z3db0y.davidlib.DriveTrain;
import com.z3db0y.davidlib.LocationTracker;
import com.z3db0y.davidlib.Logger;
import com.z3db0y.davidlib.Motor;
import com.z3db0y.davidlib.Vector;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.*;

@TeleOp(name = "FGC22", group = "FGC22")
public class DriveNew extends LinearOpMode {
    Map<String, Motor> motors = new HashMap<>();
    List<Toggles> toggles = new ArrayList<>();
    DriveTrain driveTrain;
    LocationTracker tracker;
    Vector location;
    Vector sinkCenterLocation = Configurable.sinkCenterLocation;
    double SHOOTING_CIRCLE_RADIUS = 250;
    double minimumDistanceToTarget;
    BNO055IMU imu;

    enum Toggles {
//        SLIDE,
        SHOOTER,
        COLLECTOR;

        public double prevState = 0;
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

        motors.get("shooterDown").setDirection(DcMotorSimple.Direction.REVERSE);
        motors.get("shooterUp").setDirection(DcMotorSimple.Direction.REVERSE);

        Map<DriveTrain.Location, Motor> driveTrainMotors = new HashMap<>();
        driveTrainMotors.put(DriveTrain.Location.BACKLEFT, motors.get("leftSide"));
        driveTrainMotors.put(DriveTrain.Location.BACKRIGHT, motors.get("rightSide"));
        driveTrain = new DriveTrain(DriveTrain.Type.TWOWHEELDRIVE, driveTrainMotors);
    }

    void initLocator() {
        tracker = new LocationTracker();
        location = tracker.getPosition();
        location.X = 25;
        location.Y = 33;
        location.Z = 25;

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
        toggles.add(Toggles.COLLECTOR);
        toggles.add(Toggles.SHOOTER);
//        toggles.add(Toggles.SLIDE);
        Logger.setTelemetry(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));
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
                if(toggles.contains(toggle)) toggles.remove(toggle);
                else toggles.add(toggle);
            }

            public boolean check(Object cond, Toggles toggle) {
                boolean pass = false;
                if(cond instanceof Number) pass = ((Number) cond).doubleValue() > 0;
                else if(cond instanceof Boolean) pass = (boolean) cond;

                return pass && toggle.prevState == 0;
            }
        }

        Toggler toggler = new Toggler();
        if(toggler.check(gamepad1.a, Toggles.SHOOTER)) {
            toggler.toggle(Toggles.SHOOTER);
        }
        Toggles.SHOOTER.prevState = gamepad1.a ? 1 : 0;
        Toggles.COLLECTOR.prevState = gamepad1.b ? 1 : 0;
//        Toggles.SLIDE.prevState = gamepad2.b ? 1 : 0;
    }

    void checkShooterValidity() {
        if(toggles.contains(Toggles.SHOOTER)) {
            if(!tracker.checkInsideCircle(sinkCenterLocation, SHOOTING_CIRCLE_RADIUS)) {
                Logger.addDataDashboard("insideShootingCircle", "false");
                toggles.remove(Toggles.SHOOTER);
            } else Logger.addDataDashboard("insideShootingCircle", "true");
        }
    }

    void shooter() {
        checkShooterValidity();

        PIDFCoefficients shooterUpPID = Configurable.shooterUpCoeffs;
        PIDFCoefficients shooterDownPID = Configurable.shooterDownCoeffs;
        double velocity = this.calculateShooterVelocity();
        Logger.addDataDashboard("TARGETVELORAD", velocity);


        motors.get("shooterUp").setPIDFCoefficients(RunMode.RUN_USING_ENCODER, shooterUpPID);
        motors.get("shooterDown").setPIDFCoefficients(RunMode.RUN_USING_ENCODER, shooterDownPID);

        if(toggles.contains(Toggles.SHOOTER)) {
            motors.get("shooterUp").setVelocity(velocity, AngleUnit.RADIANS);
            motors.get("shooterDown").setVelocity(velocity, AngleUnit.RADIANS);
        }
        else {
            motors.get("shooterUp").setPower(0);
            motors.get("shooterDown").setPower(0);
        }
    }

    double calculateShooterVelocity() {
        double shooterGearRatio = Configurable.shooterGearRatio;
        double shooterWheelRadiusStart = Configurable.shooterWheelRadiusStart;
        double shooterWheelMaxExpansion = Configurable.shooterWheelMaxExpansion;
        double shooterMaxVelo = Configurable.shooterMaxVelo;
        double verticalDistanceToTarget = sinkCenterLocation.Y - location.Y;
        double horizontalDistanceToTarget = tracker.distanceTo(sinkCenterLocation);
        Logger.addDataDashboard("horizontalDistanceToTarget", horizontalDistanceToTarget);
        Logger.addDataDashboard("veloUp", motors.get("shooterUp").getVelocity());
//        double shooterAngleToTarget = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle + 90; // control hub is 90 degrees off the way it is placed
        double shooterAngleToTarget = 35.0;
        double shooterAngleToTargetTan = Math.tan(Math.toRadians(shooterAngleToTarget));
        // D * targetAngleTan has to be bigger/equal(risky) than verticalDistanceToTarget
        minimumDistanceToTarget = verticalDistanceToTarget / shooterAngleToTargetTan;
        double sqrdHorizontalDistanceToTarget = Math.pow(horizontalDistanceToTarget, 2);
        double sqrdTargetAngleTan = Math.pow(shooterAngleToTargetTan, 2);
        Acceleration gravity = imu.getGravity();
        double gravityMagnitude = Math.sqrt(Math.pow(gravity.xAccel, 2) + Math.pow(gravity.yAccel, 2) + Math.pow(gravity.zAccel, 2));

        Logger.addData("Shooter angle to target: " + shooterAngleToTarget);

        double targetVelo = Math.sqrt(gravityMagnitude * sqrdHorizontalDistanceToTarget * (1 + sqrdTargetAngleTan) /
                (2 * (horizontalDistanceToTarget * shooterAngleToTargetTan - verticalDistanceToTarget)));
        double shooterWheelRadius = shooterWheelRadiusStart + (shooterWheelMaxExpansion * motors.get("shooterUp").getVelocity(AngleUnit.RADIANS) / shooterMaxVelo);
        double targetVeloRad = targetVelo / shooterGearRatio / shooterWheelRadius;

        Logger.addDataDashboard("targetVelo / shooterGearRatio / shooterWheelRadius: " + targetVelo + "/" , shooterGearRatio + "/" + shooterWheelRadius + "=" + targetVeloRad);
        return targetVeloRad;
    }

    void collector() {
        motors.get("collector").setPower(1);
//       motors.get("collector").setPower(toggles.contains(Toggles.COLLECTOR) ? 1 : 0);
        if(toggles.contains(Toggles.COLLECTOR)) {
            motors.get("collector").setPower(1);
        }
        else {
            motors.get("collector").setPower(0);
        }
    }

    void slide() {
//        if(toggles.contains(Toggles.SLIDE)) {
//            motors.get("slide").setPower(1);
//        }
//        else {
//            motors.get("slide").setPower(0);
//        }
    }

    void conveyor() {
        motors.get("conveyor").setPower(-1);
    }

    void logging() {
        Logger.addData("Powers:");
//        Logger.addDataDashboard("|--  GlobalPowerFactor: " , globalPowerFactor);
        Logger.addDataDashboard("|--  Right Side power: " , motors.get("rightSide").getPower());
        Logger.addDataDashboard("|--  Left Side power: " , motors.get("leftSide").getPower());
        Logger.addDataDashboard("|--  Conveyor power: " , motors.get("conveyor").getPower());
        Logger.addDataDashboard("|--  Collector power: " , motors.get("collector").getPower());
        Logger.addDataDashboard("|--  ShooterUp power: " , motors.get("shooterUp").getPower());
        Logger.addDataDashboard("|--  ShooterDown power: " , motors.get("shooterDown").getPower());
        Logger.addDataDashboard("|--  ShooterUp velocity in radians: " , motors.get("shooterUp").getVelocity(AngleUnit.RADIANS));
        Logger.addDataDashboard("|--  ShooterDown velocity in radians: " , motors.get("shooterDown").getVelocity(AngleUnit.RADIANS));
        Logger.addData("Ticks:");
        Logger.addDataDashboard("|--  RightSide ticks: " , motors.get("rightSide").getCurrentPosition());
        Logger.addDataDashboard("|--  LeftSide ticks: " , motors.get("leftSide").getCurrentPosition());
        Logger.addDataDashboard("|--  Conveyor ticks: " , motors.get("conveyor").getCurrentPosition());
        Logger.addDataDashboard("|--  collector ticks: " , motors.get("collector").getCurrentPosition());
        Logger.addDataDashboard("|--  ShooterUp ticks: " , motors.get("shooterUp").getCurrentPosition());
        Logger.addDataDashboard("|--  ShooterDown ticks: " , motors.get("shooterDown").getCurrentPosition());
        Logger.addData("Info (usually variables):");
//        Logger.addDataDashboard("|--  shooter step: " , shooterStep);
//        Logger.addDataDashboard("|--  prevTime: " , prevTime);
//        Logger.addDataDashboard("|--  ShooterUp target velocity in radians: " , targetVeloRad);
//        Logger.addDataDashboard("|--  ShooterDown target velocity in radians: " , targetVeloRad);
        Logger.addData("Position:");
        Logger.addDataDashboard("|--  X: " , tracker.getPosition().X);
        Logger.addDataDashboard("|--  Y: " , tracker.getPosition().Y);
        Logger.addDataDashboard("|--  Z: " , tracker.getPosition().Z);
        Logger.addDataDashboard("|--  targetAngle: " , tracker.angleToTarget(sinkCenterLocation));
        Logger.addDataDashboard("|--  DistanceToTarget center: ", tracker.distanceTo(sinkCenterLocation));
        Logger.addDataDashboard("|--  currentAngle: " , -imu.getAngularOrientation().firstAngle);
        Logger.addData("Booleans: ");
        Logger.addDataDashboard("|-- insideShootingCircle: ", tracker.checkInsideCircle(sinkCenterLocation, SHOOTING_CIRCLE_RADIUS));
        Logger.addDataDashboard("|-- Inside no-man's land: ", tracker.checkInsideCircle(sinkCenterLocation, minimumDistanceToTarget));
//        Logger.addDataDashboard("|-- shooterActivated: ", shooterActivated);
        Logger.update();
    }

    void vectorControl() {
        if(gamepad1.dpad_down) {
            location.X = 50;
            location.Z = 350;
        }
    }

    @Override
    public void runOpMode() {
        initSelf();
        waitForStart();
        while (opModeIsActive()) {
            track();
            vectorControl();

            drive(gamepad1.left_stick_y, gamepad1.right_stick_x);
            shooter();
            collector();
            slide();
            conveyor();

            toggles();
            logging();
        }
    }
}
