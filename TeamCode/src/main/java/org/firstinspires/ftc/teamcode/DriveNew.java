package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.z3db0y.davidlib.DriveTrain;
import com.z3db0y.davidlib.LocationTracker;
import com.z3db0y.davidlib.Logger;
import com.z3db0y.davidlib.Motor;
import com.z3db0y.davidlib.Vector;

import java.util.*;

@TeleOp(name = "FGC22", group = "FGC22")
public class DriveNew extends LinearOpMode {
    Map<String, Toggles>[] keyMap = new Map[]{
            // Gamepad 1
            new HashMap<String, Toggles>() {{
                put("right_bumper", Toggles.SHOOTER);
                put("circle", Toggles.COLLECTOR);
            }},
            // Gamepad 2
            new HashMap<String, Toggles>() {{
                put("dpad_up", Toggles.SLIDE_UP);
                put("dpad_down", Toggles.SLIDE_DOWN);
                put("right_trigger", Toggles.HOOK_UP);
                put("left_trigger", Toggles.HOOK_DOWN);
                put("triangle", Toggles.CONVEYOR);
            }}
    };

    Map<String, Motor> motors = new HashMap<>();
    List<Toggles> toggles = new ArrayList<>();
    DriveTrain driveTrain;
    LocationTracker tracker;
    Vector sinkCenterLocation = Configurable.sinkCenterLocation;
    double SHOOTING_CIRCLE_RADIUS = 250;
    double minimumDistanceToTarget;
    BNO055IMU imu;

    enum Toggles {
        HOOK_UP,
	    HOOK_DOWN,
        SLIDE_UP,
	    SLIDE_DOWN,
        CONVEYOR,
        SHOOTER,
        COLLECTOR,
        EATING;

        public boolean prevState;
    }

    void initMotors() {
        motors.put("leftSide", new Motor(hardwareMap.get(DcMotorImplEx.class, "leftSide")));
        motors.put("rightSide", new Motor(hardwareMap.get(DcMotorImplEx.class, "rightSide")));
        motors.put("shooterUp", new Motor(hardwareMap.get(DcMotorImplEx.class, "shooterUp")));
        motors.put("shooterDown", new Motor(hardwareMap.get(DcMotorImplEx.class, "shooterDown")));
        motors.put("conveyor", new Motor(hardwareMap.get(DcMotorImplEx.class, "conveyor")));
        motors.put("collector", new Motor(hardwareMap.get(DcMotorImplEx.class, "collector")));
        motors.put("slide", new Motor(hardwareMap.get(DcMotorImplEx.class, "slide")));
        motors.put("hook", new Motor(hardwareMap.get(DcMotorImplEx.class, "hook")));

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
        tracker.currentLocation.X = Configurable.robotDepth / 2;
        tracker.currentLocation.Y = Configurable.robotHeight / 2 + Configurable.centerToShooter;
        tracker.currentLocation.Z = Configurable.robotWidth / 2;

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
//        toggles.add(Toggles.SHOOTER);
//        toggles.add(Toggles.SLIDE);
//        toggles.add(Toggles.HOOK);
        toggles.add(Toggles.CONVEYOR);
        Logger.setTelemetry(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));
        this.initMotors();
        this.initLocator();
        this.initIMU();
    }

    void track() {
        tracker.updatePosition(imu.getAngularOrientation().firstAngle);
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

                boolean valid = pass && !toggle.prevState;

                if(valid) {
                    toggle.prevState = true;
                } else if(!pass) {
                    toggle.prevState = false;
                }

                return valid;
            }
        }

        Toggler toggler = new Toggler();
        try {
            for (int i = 0; i < 2; i++) {
                Gamepad gamepad = i == 0 ? gamepad1 : gamepad2;
                Map<String, Toggles> map = keyMap[i];
                for (String key : map.keySet()) {
                    Toggles toggle = map.get(key);
                    Logger.addDataDashboard("Toggle " + toggle, toggles.contains(toggle));
                    if (toggler.check(gamepad.getClass().getField(key).get(gamepad), toggle)) {
                        toggler.toggle(toggle);
                    }
                }
            }
        } catch (Exception ignored) {}
    }

    void checkShooterValidity() {
        if(toggles.contains(Toggles.SHOOTER)) {
            if(!tracker.checkInsideCircle(sinkCenterLocation, SHOOTING_CIRCLE_RADIUS)) {
                Logger.addDataDashboard("insideShootingCircle", "false");
                toggles.remove(Toggles.SHOOTER);
            }
            else Logger.addDataDashboard("insideShootingCircle", "true");
        }
    }

    void shooter() {
        checkShooterValidity();

//        PIDFCoefficients shooterUpPID = Configurable.shooterUpCoeffs;
//        PIDFCoefficients shooterDownPID = Configurable.shooterDownCoeffs;
        double velocity = this.calculateShooterVelocity();
        Logger.addDataDashboard("TARGET VELOCITY (ticks/sec)", velocity);

        if(toggles.contains(Toggles.SHOOTER)) {
            motors.get("shooterUp").setVelocity(velocity);
            motors.get("shooterDown").setVelocity(velocity);
        }
        else {
            motors.get("shooterUp").setPower(0);
            motors.get("shooterDown").setPower(0);
        }
    }

//    double calculateShooterVelocity() {
//        double shooterGearRatio = Configurable.shooterGearRatio;
//        double shooterWheelRadiusStart = Configurable.shooterWheelRadiusStart;
//        double shooterWheelMaxExpansion = Configurable.shooterWheelMaxExpansion;
//        double shooterMaxVelo = Configurable.shooterMaxVelo;
//        double verticalDistanceToTarget = sinkCenterLocation.Y - location.Y;
//        double horizontalDistanceToTarget = tracker.distanceTo(sinkCenterLocation);
//        Logger.addDataDashboard("horizontalDistanceToTarget", horizontalDistanceToTarget);
//        Logger.addDataDashboard("veloUp", motors.get("shooterUp").getVelocity());
////        double shooterAngleToTarget = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle + 90; // control hub is 90 degrees off the way it is placed
//        double shooterAngleToTarget = 69.0;
//        double shooterAngleToTargetTan = Math.tan(Math.toRadians(shooterAngleToTarget));
//        // D * targetAngleTan has to be bigger/equal(risky) than verticalDistanceToTarget
//        minimumDistanceToTarget = verticalDistanceToTarget / shooterAngleToTargetTan;
//        double sqrdHorizontalDistanceToTarget = Math.pow(horizontalDistanceToTarget, 2);
//        double sqrdTargetAngleTan = Math.pow(shooterAngleToTargetTan, 2);
//        Acceleration gravity = imu.getGravity();
//        double gravityMagnitude = Math.sqrt(Math.pow(gravity.xAccel, 2) + Math.pow(gravity.yAccel, 2) + Math.pow(gravity.zAccel, 2));
//
//        Logger.addData("Shooter angle to target: " + shooterAngleToTarget);
//
//        double targetVelo = Math.sqrt(gravityMagnitude * sqrdHorizontalDistanceToTarget * (1 + sqrdTargetAngleTan) /
//                (2 * (horizontalDistanceToTarget * shooterAngleToTargetTan - verticalDistanceToTarget)));
//        double shooterWheelRadius = shooterWheelRadiusStart + (shooterWheelMaxExpansion * motors.get("shooterUp").getVelocity(AngleUnit.RADIANS) / shooterMaxVelo);
//        double targetVeloRad = targetVelo / shooterGearRatio / shooterWheelRadius;
//
//        Logger.addDataDashboard("targetVelo / shooterGearRatio / shooterWheelRadius: " + targetVelo + "/" , shooterGearRatio + "/" + shooterWheelRadius + "=" + targetVeloRad);
//        return targetVeloRad;
//    }

    double calculateShooterVelocity() {
        double shooterGearRatio = Configurable.shooterGearRatio;
        double shooterWheelRadiusStart = Configurable.shooterWheelRadiusStart;
        double shooterWheelMaxExpansion = Configurable.shooterWheelMaxExpansion;
        double shooterMaxVelo = Configurable.shooterMaxVelo;

        double shooterWheelRadius = shooterWheelRadiusStart + (shooterWheelMaxExpansion * Math.abs(motors.get("shooterUp").getVelocity()) / shooterMaxVelo);

        double verticalDistanceToTarget = (sinkCenterLocation.Y - tracker.currentLocation.Y + Configurable.ballDiameter) / 100; //maybe add 60/2???
        double horizontalDistanceToTarget = (tracker.distanceTo(sinkCenterLocation) - 60) / 100;

        double angleToTarget = Configurable.shooterAngle;
        double gravity = Math.sqrt(
                Math.pow(imu.getGravity().xAccel, 2) +
                Math.pow(imu.getGravity().yAccel, 2) +
                Math.pow(imu.getGravity().zAccel, 2)
        );

        double paragondas = Configurable.paragondas;

//        double targetDistanceVelo = 0.8 * (Math.tan(Math.toRadians(angleToTarget)) * horizontalDistanceToTarget + (gravity * Math.pow(horizontalDistanceToTarget, 2) / 2 * verticalDistanceToTarget * Math.cos(Math.toRadians(angleToTarget))));
        double targetDistanceVelo = paragondas * Math.sqrt(gravity * Math.pow(horizontalDistanceToTarget, 2) * (1 + Math.pow(Math.tan(Math.toRadians(angleToTarget)), 2)) /
                (2 * (horizontalDistanceToTarget * Math.tan(Math.toRadians(angleToTarget)) - verticalDistanceToTarget)));
        Logger.addDataDashboard("TDV", targetDistanceVelo);

        return (targetDistanceVelo * 100) / (2 * Math.PI * shooterWheelRadius) * (28 / shooterGearRatio); // ticks/sec
    }

    void collector() {
        if(toggles.contains(Toggles.COLLECTOR)) {
            motors.get("collector").setPower(-1);
        }
        else {
            motors.get("collector").setPower(0);
        }
    }

    void slide() {
        if(toggles.contains(Toggles.SLIDE_UP)) {
            motors.get("slide").setPower(1);
        }
	else if(toggles.contains(Toggles.SLIDE_DOWN)) {
            motors.get("slide").setPower(-1);
	}
        else {
            motors.get("slide").setPower(0);
        }
    }

    void hook() {
        if(gamepad2.right_trigger > 0) {
            motors.get("hook").setPower(gamepad2.right_trigger);
        }
        else if(gamepad2.left_trigger > 0) {
                motors.get("hook").setPower(-gamepad2.left_trigger);
        }
        else {
            motors.get("hook").setPower(0);
        }
    }

    void conveyor() {
//        double avgVelo = motors.get("shooterUp").getVelocity() + motors.get("shooterDown").getVelocity() / 2;
//        if
        if(toggles.contains(Toggles.CONVEYOR)) {
            motors.get("conveyor").setPower(-1);
        }
        else {
            motors.get("conveyor").setPower(0);
        }
    }

    void logging() {
        Logger.addData("Powers:");
        Logger.addDataDashboard("|--  Right Side power: " , motors.get("rightSide").getPower());
        Logger.addDataDashboard("|--  Left Side power: " , motors.get("leftSide").getPower());
        Logger.addDataDashboard("|--  Conveyor power: " , motors.get("conveyor").getPower());
        Logger.addDataDashboard("|--  Collector power: " , motors.get("collector").getPower());
        Logger.addDataDashboard("|--  ShooterUp power: " , motors.get("shooterUp").getPower());
        Logger.addDataDashboard("|--  ShooterDown power: " , motors.get("shooterDown").getPower());
        Logger.addDataDashboard("|--  Slide power: " , motors.get("slide").getPower());
        Logger.addDataDashboard("|--  Hook power: " , motors.get("hook").getPower());
        Logger.addDataDashboard("|--  ShooterUp velocity: " , motors.get("shooterUp").getVelocity());
        Logger.addDataDashboard("|--  ShooterDown velocity: " , motors.get("shooterDown").getVelocity());
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
        Logger.update();
    }

    void vectorControl() {
        if(gamepad1.dpad_down) {
            tracker.currentLocation.X = 75;
            tracker.currentLocation.Z = 350;
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
            hook();
            conveyor();

            toggles();
            logging();
        }
    }
}
