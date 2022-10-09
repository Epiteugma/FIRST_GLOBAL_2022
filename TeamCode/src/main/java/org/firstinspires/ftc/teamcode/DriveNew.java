package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import com.z3db0y.davidlib.DriveTrain;
import com.z3db0y.davidlib.LocationTracker;
import com.z3db0y.davidlib.Logger;
import com.z3db0y.davidlib.Motor;
import com.z3db0y.davidlib.Vector;

import java.util.*;

@TeleOp(name = "FGC22", group = "FGC22")
public class DriveNew extends LinearOpMode {

    class Toggler {

        public void toggle(Toggles toggle) {
            if(toggles.contains(toggle)) toggles.remove(toggle);
            else toggles.add(toggle);
        }

        public boolean check(Object cond, Toggles toggle) {
            boolean pass = false;
            if(cond instanceof Number) pass = ((Number) cond).doubleValue() > 0.15;
            else if(cond instanceof Boolean) pass = (boolean) cond;

            boolean valid = pass && !toggle.prevState;

            if(valid) {
                toggle.prevState = true;
            }
            else if(!pass) {
                toggle.prevState = false;
            }

            return valid;
        }
    }
    Toggler toggler = new Toggler();

    Map<String, Toggles>[] keyMap = new Map[]{
            // Gamepad 1
            new HashMap<String, Toggles>() {{
                put("dpad_up", Toggles.INCREASE_SHOOTER);
                put("dpad_down", Toggles.DECREASE_SHOOTER);
                put("dpad_right", Toggles.RESET_RIGHT);
                put("dpad_left", Toggles.RESET_LEFT);
                put("right_bumper", Toggles.SHOOTER);
                put("square", Toggles.FREEZE_SLIDE);
                put("guide", Toggles.CENTER_SHOT);
//                put("cross", Toggles.FACE_TARGET);
            }},
            // Gamepad 2
            new HashMap<String, Toggles>() {{
                put("guide", Toggles.EMPTY);
                put("cross", Toggles.EATING);
                put("square", Toggles.FREEZE_HOOK);
                put("dpad_up", Toggles.SLIDE_UP);
                put("dpad_down", Toggles.SLIDE_DOWN);
//                put("right_trigger", Toggles.HOOK_UP); // made with default trigger sdk
//                put("left_trigger", Toggles.HOOK_DOWN); // made with default trigger sdk
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
        FREEZE_SLIDE,
        CENTER_SHOT,
        HOOK_UP,
        HOOK_DOWN,
        INCREASE_SHOOTER,
        DECREASE_SHOOTER,
        RESET_LEFT,
        RESET_RIGHT,
        EMPTY,
        FREEZE_HOOK,
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
    double currentAngle = 0;

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

//        motors.get("rightSide").setHoldPosition(true);
//        motors.get("leftSide").setHoldPosition(true);
//        motors.get("hook").setHoldPosition(true);
        motors.get("slide").setHoldPosition(true);


        motors.get("rightSide").setDirection(DcMotorSimple.Direction.REVERSE);
        motors.get("leftSide").setDirection(DcMotorSimple.Direction.REVERSE);
        motors.get("shooterDown").setDirection(DcMotorSimple.Direction.REVERSE);
//        motors.get("shooterUp").setDirection(DcMotorSimple.Direction.REVERSE);
        motors.get("hook").setDirection(DcMotorSimple.Direction.REVERSE);
        motors.get("collector").setDirection(DcMotorSimple.Direction.REVERSE);
        motors.get("conveyor").setDirection(DcMotorSimple.Direction.REVERSE);

        motors.get("hook").setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Map<DriveTrain.Location, Motor> driveTrainMotors = new HashMap<>();
        driveTrainMotors.put(DriveTrain.Location.BACKLEFT, motors.get("leftSide"));
        driveTrainMotors.put(DriveTrain.Location.BACKRIGHT, motors.get("rightSide"));
        driveTrain = new DriveTrain(DriveTrain.Type.TWOWHEELDRIVE, driveTrainMotors);
    }

    void initLocator() {
        tracker = new LocationTracker();
        tracker.currentLocation.X = Configurable.robotDepth / 2 + Configurable.compressorLength; // add compressor
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
        currentAngle = imu.getAngularOrientation().firstAngle;
        double averageVelo = (motors.get("leftSide").getMotorType().getAchieveableMaxTicksPerSecond() * motors.get("leftSide").getPower()) +
                (motors.get("rightSide").getMotorType().getAchieveableMaxTicksPerSecond() * motors.get("rightSide").getPower()) / 2;
        double xVelo = -imu.getAngularVelocity().xRotationRate/(2 * Math.PI) * Configurable.motorTicksPerRevolution;
        Logger.addDataDashboard("xVelo", xVelo);
        Logger.addDataDashboard("averageVelo", averageVelo);
        tracker.updatePosition(currentAngle);
    }

    void drive(double left_stick_y, double right_stick_x) {
        driveTrain.driveRobotCentric(left_stick_y, right_stick_x, 0);
    }

    void toggles() {
        try {
            for (int i = 0; i < 2; i++) {
                Gamepad gamepad = i == 0 ? gamepad1 : gamepad2;
                Map<String, Toggles> map = keyMap[i];
                for (String key : map.keySet()) {
                    Toggles toggle = map.get(key);
                    if (toggler.check(gamepad.getClass().getField(key).get(gamepad), toggle)) {
                        toggler.toggle(toggle);
                    }
                }
            }
        }
        catch (Exception ignored) {}
    }

    void checkShooterValidity() {
        if(toggles.contains(Toggles.SHOOTER)) {
            if(!tracker.checkInsideCircle(sinkCenterLocation, SHOOTING_CIRCLE_RADIUS)) {
                Logger.addDataDashboard("insideShootingCircle", "false");
                gamepad1.rumble(1000);
                toggles.remove(Toggles.SHOOTER);
            }
            else {
//                Logger.speak("Inside Shooting Circle");
                Logger.addDataDashboard("insideShootingCircle", "true");
            }
        }
    }

    void shooter() {

        checkShooterValidity();

        if(toggles.contains(Toggles.INCREASE_SHOOTER)){
            toggles.remove(Toggles.INCREASE_SHOOTER);
            tracker.currentLocation.X -= 5;
            telemetry.speak("Increased velocity");
        }
        else if(toggles.contains(Toggles.DECREASE_SHOOTER)){
            toggles.remove(Toggles.DECREASE_SHOOTER);
            tracker.currentLocation.X += 5;
            telemetry.speak("Decreased Velocity");
        }

        Logger.addDataDashboard("TARGET VELOCITY (ticks/sec)", shooterVelocity);
        if(toggles.contains(Toggles.EATING) || toggles.contains(Toggles.EMPTY)) {
            motors.get("shooterUp").setVelocity(-220);
            motors.get("shooterDown").setVelocity(-220);
        }
        else if(toggles.contains(Toggles.SHOOTER)) {
            shooterVelocity = this.calculateShooterVelocity();
            motors.get("shooterUp").setVelocity(shooterVelocity);
            motors.get("shooterDown").setVelocity(shooterVelocity);
            if(motors.get("shooterUp").getVelocity() < shooterVelocity) {
                Logger.addDataDashboard("UP CANT REACH TARGET VELOCITY", "true");
            }
            if(motors.get("shooterDown").getVelocity() < shooterVelocity){
                Logger.addDataDashboard("DOWN CANT REACH TARGET VELOCITY", "true");
            }
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

        double verticalDistanceToTarget = (sinkCenterLocation.Y - tracker.currentLocation.Y + Configurable.ballDiameter) / 100; //maybe add 60/2??? and not Configurable.ballDiameter
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
        if (toggles.contains(Toggles.EMPTY)){
            motors.get("collector").setPower(-1);
        }
        else if(toggles.contains(Toggles.COLLECTOR) || toggles.contains(Toggles.EATING)) {
            motors.get("collector").setPower(1);
        }
        else {
            motors.get("collector").setPower(0);
        }
    }

//    void faceTarget(){
//        if(toggles.contains(Toggles.FACE_TARGET)){
//            toggles.remove(Toggles.FACE_TARGET);
//            double targetAngle = tracker.currentLocation.anglesTo(sinkCenterLocation).get(0);
//
//            driveTrain.turn(0.3, targetAngle, Configurable.wheelRadius, Configurable.motorTicksPerRevolution, Configurable.robotWidth);
//        }
//    }

    void slide() {
        int slideMaxDistance = Configurable.slideMaxDistance;
        int hookMaxDistance = Configurable.hookMaxDistance;
        double slideCirc = 2 * Math.PI * slideRadius;
        Logger.addDataDashboard("slideDistance", slideDistance);
//        if(toggles.contains(Toggles.FREEZE_SLIDE)){
//            motors.get("slide").setMode(RunMode.RUN_USING_ENCODER);
//            toggles.remove(Toggles.SLIDE_UP);
//            toggles.remove(Toggles.SLIDE_DOWN);
//            toggles.remove(Toggles.FREEZE_SLIDE);
//            motors.get("slide").setHoldPosition(true);
//        }
//        if(toggles.contains(Toggles.SLIDE_DOWN)){
//            toggles.remove(Toggles.SLIDE_UP);
//            toggles.remove(Toggles.SLIDE_DOWN);
//            motors.get("slide").setPower(-0.75);
//        }
//        else if(toggles.contains(Toggles.SLIDE_UP)) {
//            toggles.remove(Toggles.SLIDE_DOWN);
//            toggles.remove(Toggles.SLIDE_UP);
//            motors.get("slide").setPower(0.35);
//        }
//        else{
//            motors.get("slide").setPower(0);
//        }
        if(gamepad2.dpad_up) motors.get("slide").setPower(1);
        else if(gamepad2.dpad_down) motors.get("slide").setPower(-1);
        else motors.get("slide").setPower(0);
    }

    void hook() {
        if(toggles.contains(Toggles.FREEZE_HOOK)){
            toggles.remove(Toggles.HOOK_UP);
            toggles.remove(Toggles.HOOK_DOWN);
            motors.get("hook").setPower(0);
            motors.get("hook").setHoldPosition(true);
        }
        if(Math.abs(gamepad2.right_trigger) > 0.1){
            toggles.remove(Toggles.HOOK_DOWN);
            motors.get("hook").setMode(RunMode.RUN_USING_ENCODER);
            motors.get("hook").setPower(gamepad2.right_trigger);
        }
        else if(Math.abs(gamepad2.left_trigger) > 0.1){
            toggles.remove(Toggles.HOOK_UP);
            motors.get("hook").setMode(RunMode.RUN_USING_ENCODER);
            motors.get("hook").setPower(-gamepad2.left_trigger);
        }
        else {
            Logger.addDataDashboard("Hook holdingPos", "true");
            motors.get("hook").setPower(0);
        }
    }

    void conveyor() {
        double avgVelo = motors.get("shooterUp").getVelocity() + motors.get("shooterDown").getVelocity() / 2;
        if(toggles.contains(Toggles.EMPTY)){
            toggles.remove(Toggles.EATING);
        }
        if(toggles.contains(Toggles.EATING)){
            toggles.remove(Toggles.EMPTY);
        }
        if (toggles.contains(Toggles.CONVEYOR)) {
            if(toggles.contains(Toggles.EATING) || toggles.contains(Toggles.EMPTY)) motors.get("conveyor").setPower(-1);
            else motors.get("conveyor").setPower(1);
        }
        else {
            motors.get("conveyor").setPower(0);
        }
    }

    void logging() {
        motors.forEach((name, motor) -> {
            Logger.addData(name + ": ");
            Logger.addDataDashboard("|--  power: ", motor.getPower());
            Logger.addDataDashboard("|--  velocity: ", motor.getVelocity());
            Logger.addDataDashboard("|--  ticks: ", motor.getCurrentPosition());
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
        Logger.addDataDashboard("|--  currentAngle: " , currentAngle);
        Logger.addData("Booleans: ");
        Logger.addDataDashboard("|-- insideShootingCircle: ", tracker.checkInsideCircle(sinkCenterLocation, SHOOTING_CIRCLE_RADIUS));
        Logger.addDataDashboard("|-- Inside no-man's land: ", tracker.checkInsideCircle(sinkCenterLocation, minimumDistanceToTarget));
        Logger.update();
    }

    void vectorControl() {
        if(toggles.contains(Toggles.CENTER_SHOT)){
            toggles.remove(Toggles.CENTER_SHOT);
            tracker.currentLocation.X = Configurable.robotDepth / 2;
            tracker.currentLocation.Y = Configurable.robotHeight / 2 + Configurable.centerToShooter;
            tracker.currentLocation.Z = 350;
        }
        if(toggles.contains(Toggles.RESET_LEFT)){
            toggles.remove(Toggles.RESET_LEFT);
//            imu.resetHeading();
            tracker.currentLocation.X = Configurable.robotDepth / 2;
            tracker.currentLocation.Y = Configurable.robotHeight / 2 + Configurable.centerToShooter;
            tracker.currentLocation.Z = Configurable.robotWidth / 2 + Configurable.compressorLength; // add compressor
            telemetry.speak("Reset Left");
        }
        if(toggles.contains(Toggles.RESET_RIGHT)){
            toggles.remove(Toggles.RESET_RIGHT);
            tracker.currentLocation.X = Configurable.robotDepth / 2;
            tracker.currentLocation.Y = Configurable.robotHeight / 2 + Configurable.centerToShooter;
            tracker.currentLocation.Z = 700 - Configurable.compressorLength - Configurable.robotWidth / 2; // subtract compressor
            telemetry.speak("Reset Right");
        }
    }

    @Override
    public void runOpMode() {
        initSelf();
        waitForStart();

        new Thread(() -> { while(opModeIsActive()) {
            toggles();
            drive(gamepad1.left_stick_y, -gamepad1.right_stick_x);
//            drive(gamepad1.left_stick_y * Configurable.forwardPowerFactor, Range.clip(-gamepad1.right_stick_x, -0.7, 0.7) * Configurable.turnPowerFactor);
        }}).start();

        while (opModeIsActive()) {
            vectorControl();
            track();

            shooter();
            conveyor();
            collector();
            hook();
            slide();

            logging();
        }
    }
}
