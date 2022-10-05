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
                put("cross", Toggles.FACE_TARGET);
            }},
            // Gamepad 2
            new HashMap<String, Toggles>() {{
                put("dpad_up", Toggles.SLIDE_UP);
                put("dpad_down", Toggles.SLIDE_DOWN);
                // put("right_trigger", Toggles.HOOK_UP);
                // put("left_trigger", Toggles.HOOK_DOWN);
                put("triangle", Toggles.CONVEYOR);
                put("circle", Toggles.COLLECTOR);
            }}
    };

    Map<String, Motor> motors = new HashMap<>();
    List<Toggles> toggles = new ArrayList<>();
    DriveTrain driveTrain;
    LocationTracker tracker;
    Vector sinkCenterLocation = Configurable.sinkCenterLocation;
    double SHOOTING_CIRCLE_RADIUS = Configurable.SHOOTING_CIRCLE_RADIUS;
    double minimumDistanceToTarget;
    BNO055IMU imu;

    enum Toggles {
        // HOOK_UP,
	    // HOOK_DOWN,
        FACE_TARGET,
        SLIDE_UP,
        SLIDE_DOWN,
        CONVEYOR,
        SHOOTER,
        COLLECTOR,
        EATING;

        public boolean prevState;
    }

    double shooterVelocity = 0;
    double slideDistance = 0;
    double hookDistance = 0;
    double slideRadius = Configurable.slideRadius;
    double hookRadius = Configurable.hookRadius;

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

//        motors.get("shooterDown").setDirection(DcMotorSimple.Direction.REVERSE);
        motors.get("shooterUp").setDirection(DcMotorSimple.Direction.REVERSE);

        motors.get("hook").setHoldPosition(true);
        motors.get("slide").setHoldPosition(true);

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
                if(cond instanceof Number) pass = ((Number) cond).doubleValue() > 0.2;
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

        shooterVelocity = this.calculateShooterVelocity();
        Logger.addDataDashboard("TARGET VELOCITY (ticks/sec)", shooterVelocity);

        if(toggles.contains(Toggles.SHOOTER)) {
            motors.get("shooterUp").setVelocity(shooterVelocity);
            motors.get("shooterDown").setVelocity(shooterVelocity);
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

    void faceTarget(){
        if(toggles.contains(Toggles.FACE_TARGET)){
            toggles.remove(Toggles.FACE_TARGET);
            double targetAngle = tracker.currentLocation.anglesTo(sinkCenterLocation).get(0);

            driveTrain.turn(0.3, targetAngle, Configurable.wheelRadius, Configurable.motorTicksPerRevolution, Configurable.robotWidth);
        }
    }

    void slide() {
        Logger.addDataDashboard("slideDistance", slideDistance);
        if(toggles.contains(Toggles.SLIDE_UP) && slideDistance < Configurable.slideMaxDistance) {
            motors.get("slide").setTargetPosition((int) Configurable.slideMaxDistance);
            motors.get("slide").setPower(1);
            motors.get("slide").setMode(RunMode.RUN_TO_POSITION);
            motors.get("hook").setPower(0.65);
            slideDistance = motors.get("slide").getCurrentPosition() / 28.0 * 2 * Math.PI * slideRadius;
        }
        else if(toggles.contains(Toggles.SLIDE_DOWN)){
            motors.get("slide").setTargetPosition(0);
            motors.get("slide").setMode(RunMode.RUN_TO_POSITION);
        }
        else {
            motors.get("slide").setPower(0);
        }
    }

    void hook() {
        if(Math.abs(gamepad2.right_trigger) > 0.2) {
            motors.get("hook").setHoldPosition(false);
            motors.get("hook").setPower(gamepad2.right_trigger);
        }
        else if(Math.abs(gamepad2.left_trigger) > 0.2) {
            motors.get("hook").setHoldPosition(false);
            motors.get("hook").setPower(-gamepad2.left_trigger);
        }
        else {
            motors.get("hook").setPower(0);
            motors.get("hook").setHoldPosition(true);
        }
    }

    void conveyor() {
        double avgVelo = motors.get("shooterUp").getVelocity() + motors.get("shooterDown").getVelocity() / 2;
        if (toggles.contains(Toggles.CONVEYOR)) {
            motors.get("conveyor").setPower(-1);
        }
        else {
            motors.get("conveyor").setPower(0);
        }
    }

    void logging() {
        motors.forEach((name, motor) -> {
            Logger.addData(name + ": ");
            Logger.addDataDashboard("|--  power: ", motor.getPower());
            Logger.addDataDashboard("|--  ticks: ", motor.getCurrentPosition());
            Logger.addDataDashboard("|--  velocity: ", motor.getVelocity());
        });
//        Logger.addData("Info (usually variables):");
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
            tracker.currentLocation.X = 200;
            tracker.currentLocation.Z = 175;
        }
    }

    @Override
    public void runOpMode() {
        initSelf();
        waitForStart();
        while (opModeIsActive()) {
            vectorControl();
            track();

            drive(gamepad1.left_stick_y * Configurable.globalPowerFactor, gamepad1.right_stick_x * Configurable.globalPowerFactor);
            shooter();
            collector();
            hook();
            slide();
            conveyor();
            faceTarget();

            toggles();
            logging();
        }
    }
}
