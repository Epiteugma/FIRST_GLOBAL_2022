package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.z3db0y.davidlib.DriveTrain;
import com.z3db0y.davidlib.LocationTracker;
import com.z3db0y.davidlib.Logger;
import com.z3db0y.davidlib.Motor;
import com.z3db0y.davidlib.Vector;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.HashMap;

@TeleOp(name = "TeleOpFGC", group = "FGC22")
public class Drive extends LinearOpMode {
    Motor leftSide;
    Motor rightSide;
    Motor conveyor;
    Motor collector;
    Motor shooterDown;
    Motor shooterUp;
    HashMap motorMap;
    BNO055IMU imu;
    DriveTrain driveTrain;
    LocationTracker tracker;
    Vector BASKET_TARGET = new Vector(300, 0, 300);
    double SHOOTING_CIRCLE_RADIUS = 250;

    public static PIDFCoefficients shooterUpCoeffs = Configurable.shooterUpCoeffs;
    public static PIDFCoefficients shooterDownCoeffs = Configurable.shooterDownCoeffs;

    public void initHardware() {
        // drivetrain
        leftSide = new Motor(hardwareMap, "leftSide");
        rightSide = new Motor(hardwareMap, "rightSide");

        // transporters
        collector = new Motor(hardwareMap, "collector");
        conveyor = new Motor(hardwareMap, "conveyor");

        // shooters
        shooterDown = new Motor(hardwareMap, "shooterDown");
        shooterUp = new Motor(hardwareMap, "shooterUp");

        // imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        // Directions
        shooterUp.setDirection(DcMotorEx.Direction.REVERSE);
        shooterDown.setDirection(DcMotorEx.Direction.REVERSE);
        leftSide.setDirection(DcMotorEx.Direction.FORWARD);
        rightSide.setDirection(DcMotorEx.Direction.FORWARD);
        collector.setDirection(DcMotorEx.Direction.FORWARD);
        conveyor.setDirection(DcMotorEx.Direction.FORWARD);

        motorMap = new HashMap<String, Motor>();
        motorMap.put("leftSide", leftSide);
        motorMap.put("rightSide", rightSide);
        motorMap.put("collector", collector);
        motorMap.put("conveyor", conveyor);
        motorMap.put("shooterUp", shooterUp);
        motorMap.put("shooterDown", shooterDown);

        // for every motor in the map, set the mode to run without encoder
        for (Object motor : motorMap.values()) {
            ((Motor) motor).setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            ((Motor) motor).setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            ((Motor) motor).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


        HashMap<DriveTrain.Location, Motor> driveTrainMotors = new HashMap<>();
        driveTrainMotors.put(DriveTrain.Location.FRONTLEFT, leftSide);
        driveTrainMotors.put(DriveTrain.Location.FRONTRIGHT, rightSide);
        driveTrain = new DriveTrain(DriveTrain.Type.TWOWHEELDRIVE, driveTrainMotors);
        tracker = new LocationTracker();
        LocationTracker.Parameters params = new LocationTracker.Parameters();
        params.driveTrain = driveTrain;
        params.motorTicksPerRevolution = Configurable.motorTicksPerRevolution;
        params.robotWidth = Configurable.robotWidth;
        params.wheelRadius = Configurable.wheelRadius;
        tracker.initialize(params);
    }

    double globalPowerFactor = 0.75;
    double collectorPower = Configurable.collectorPower;
    double conveyorPower = Configurable.conveyorPower;

    boolean shooterActivated = false;
    double shooterStep = Configurable.shooterStep;
    double shooterMarginOfError = Configurable.shooterMarginOfError;
    double minShooterVelo = Configurable.minShooterVeloRads;
    double maxShooterVelo = Configurable.maxShooterVeloRads;
    double shooterGearRatio = Configurable.shooterGearRatio;
    double shooterWheelRadiusStart = Configurable.shooterWheelRadiusStart;
    double shooterWheelMaxExpansion = Configurable.shooterWheelMaxExpansion;
    double verticalDistanceToTarget = Configurable.verticalDistanceToTarget;
    double shooterAngleToTarget;
    double minimumDistanceToTarget;

    long prevTime = 0;
    double targetVeloRadUp = 0;
    double targetVeloRadDown = 0;

    public void shooterCalculator(double horizontalDistanceToTarget) {
        shooterAngleToTarget = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle + 90; // control hub is X degrees off the way it is placed
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
//        double currentVelo = motor.getVelocity(AngleUnit.RADIANS);
        double shooterWheelRadius = shooterWheelRadiusStart + (shooterWheelMaxExpansion * targetVeloRadUp/ maxShooterVelo);
        double targetVeloRad = targetVelo / shooterGearRatio / shooterWheelRadius;

        Logger.addDataDashboard("targetVelo / shooterGearRatio / shooterWheelRadius: " + targetVelo + "/" , shooterGearRatio + "/" + shooterWheelRadius + "=" + targetVeloRad);
        Logger.addDataDashboard("targetvelo - lastargetVelo: ", Math.abs(targetVeloRad - targetVeloRadUp));
        if (Math.abs(targetVeloRad - targetVeloRadUp) > 0.125) {
            targetVeloRadUp = targetVeloRad;
            targetVeloRadDown = targetVeloRad;
        }
    }

    private void haltFIRE(){
        shooterActivated = false;
        shooterUp.setPower(0);
        shooterDown.setPower(0);
    }


    private void FIRE(){
        double horizontalDistanceToTarget = tracker.distanceToTarget(BASKET_TARGET);
        shooterCalculator(horizontalDistanceToTarget);

        shooterUp.setVelocityPIDFCoefficients(shooterUpCoeffs.p, shooterUpCoeffs.i, shooterUpCoeffs.d, shooterUpCoeffs.f);
        shooterDown.setVelocityPIDFCoefficients(shooterDownCoeffs.p, shooterDownCoeffs.i, shooterDownCoeffs.d, shooterDownCoeffs.f);
        shooterUp.setVelocity(targetVeloRadUp, AngleUnit.RADIANS);
        shooterDown.setVelocity(targetVeloRadDown, AngleUnit.RADIANS);

        if (Math.abs(shooterUp.getVelocity(AngleUnit.RADIANS)) > 9){
            haltFIRE();
            Logger.speak("Going to fast baby slow down maybe change your coefficients!");
        }

        Logger.addDataDashboard("shooterDown target velocity", targetVeloRadDown);
        Logger.addDataDashboard("shooterUp target velocity", targetVeloRadUp);
    }

    public void turn(double pow, double target) {
        double wheelCirc = 2 * Math.PI * Configurable.wheelRadius;
        double cmPer90Deg = Configurable.robotWidth * Math.PI / 2;
        double ticksPer90Deg = cmPer90Deg / wheelCirc * Configurable.motorTicksPerRevolution;
        double targetTicks = ticksPer90Deg * target / 90;
        double sideTickDelta = leftSide.getCurrentPosition() - rightSide.getCurrentPosition();
        double rotation = sideTickDelta / ticksPer90Deg * 90;
        int direction = Math.abs(target - rotation) > 180 ? 1 : -1;

        double targetLeft = leftSide.getCurrentPosition() + (targetTicks * direction);
        double targetRight = rightSide.getCurrentPosition() - (targetTicks * direction);

        leftSide.setTargetPosition((int) targetLeft);
        rightSide.setTargetPosition((int) targetRight);

        leftSide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSide.setPower(pow);
        rightSide.setPower(pow);

        while(Math.abs(leftSide.getCurrentPosition() - leftSide.getTargetPosition()) > 3 && Math.abs(leftSide.getCurrentPosition() - leftSide.getTargetPosition()) > 3) {
            Logger.addDataDashboard("targetLeft", targetLeft);
            Logger.addDataDashboard("targetRight", targetRight);
            Logger.addDataDashboard("leftSide.getCurrentPosition()", leftSide.getCurrentPosition());
            Logger.addDataDashboard("rightSide.getCurrentPosition()", rightSide.getCurrentPosition());
            Logger.addDataDashboard("rotation", rotation);
            Logger.addDataDashboard("target", target);
            Logger.addDataDashboard("direction", direction);
            Logger.addDataDashboard("sideTickDelta", sideTickDelta);
            Logger.addDataDashboard("ticksPer90Deg", ticksPer90Deg);
            Logger.addDataDashboard("targetTicks", targetTicks);
            Logger.addDataDashboard("cmPer90Deg", cmPer90Deg);
            Logger.addDataDashboard("wheelCirc", wheelCirc);
            Logger.addDataDashboard("pow", pow);
            Logger.update();
        }

        leftSide.setPower(0);
        rightSide.setPower(0);
        leftSide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    private void main(){
        double shooterStep = Configurable.shooterStep;
        double positionDiff = shooterStep * 15; // TODO: find the exact proportion of radians to distance
        if(gamepad1.right_bumper){
            targetVeloRadUp += shooterStep;
            targetVeloRadDown += shooterStep;
            tracker.setPosition(new Vector(tracker.getPosition().X - positionDiff,0,tracker.getPosition().Z));
        }
        else if(gamepad1.left_bumper){
            targetVeloRadUp -= shooterStep;
            targetVeloRadDown -= shooterStep;
            tracker.setPosition(new Vector(tracker.getPosition().X + positionDiff,0,tracker.getPosition().Z));
        }

        if(shooterActivated) FIRE();
        else haltFIRE();

        if (gamepad1.circle) {
            if (tracker.checkInsideCircle(BASKET_TARGET, SHOOTING_CIRCLE_RADIUS)) {
                if (tracker.checkInsideCircle(BASKET_TARGET, minimumDistanceToTarget)) {
                    Logger.speak("Inside minimum distance");
                }
                else {
                    shooterActivated = !shooterActivated;
                    if (!shooterActivated) Logger.speak("Shooter Deactivated");
                    else Logger.speak("Shooter activated");
                }
            }
            else{
                Logger.speak("Outside shooting circle can't shoot");
            }
        }
        else if (gamepad1.cross){
            if (!tracker.checkInsideCircle(BASKET_TARGET, SHOOTING_CIRCLE_RADIUS)) {
                Logger.speak("I am turning but I am not in the shooting circle");
            }
            double targetAngle = tracker.angleToTarget(BASKET_TARGET);

            turn(0.1, targetAngle);
        }
    }

    private void globalPowerFactorControl() {
        if (gamepad1.left_bumper && globalPowerFactor - 0.1 > 0 && System.currentTimeMillis() >= prevTime + 500) {
            prevTime = System.currentTimeMillis();
            globalPowerFactor -= 0.1;
        } else if (gamepad1.right_bumper && globalPowerFactor + 0.1 < 0.7 && System.currentTimeMillis() >= prevTime + 500) {
            prevTime = System.currentTimeMillis();
            globalPowerFactor += 0.1;
        }
    }

    private void driveTrainControl(){
        double yleft = gamepad1.left_stick_y * globalPowerFactor; //inverted
        double xright = gamepad1.right_stick_x * globalPowerFactor; //inverted
        leftSide.setPower(-yleft + xright);
        rightSide.setPower(-yleft - xright);
    }

    private void conveyorControl(){
        double upVelo = Math.abs(shooterUp.getVelocity(AngleUnit.RADIANS));
        double downVelo = Math.abs(shooterDown.getVelocity(AngleUnit.RADIANS));
        if(
                Math.abs(upVelo - targetVeloRadUp) <= shooterMarginOfError &&
                Math.abs(downVelo - targetVeloRadDown) <= shooterMarginOfError
        )
        {
            conveyor.setPower(conveyorPower);
        }
        else {
            conveyor.setPower(0);
        }
        }

    private void collectorControl() {
        collector.setPower(collectorPower);
    }

    private void locator(){
        if (gamepad1.dpad_down){
            tracker.setPosition(new Vector(150,0,150));
        }
        else if (gamepad1.dpad_left) {
            // set current position as 0,0,0
            tracker.setPosition(new Vector(0,0,0));
        }
        else if(gamepad1.dpad_right) {
            tracker.setPosition(new Vector(0,0,300));
        }
        tracker.updatePosition(-imu.getAngularOrientation().firstAngle);
    }

    private void Logging() {
        Logger.addData("Powers:");
        Logger.addDataDashboard("|--  GlobalPowerFactor: " , globalPowerFactor);
        Logger.addDataDashboard("|--  Right Side power: " , rightSide.getPower());
        Logger.addDataDashboard("|--  Left Side power: " , leftSide.getPower());
        Logger.addDataDashboard("|--  Conveyor power: " , conveyor.getPower());
        Logger.addDataDashboard("|--  Collector power: " , collector.getPower());
        Logger.addDataDashboard("|--  ShooterUp power: " , shooterUp.getPower());
        Logger.addDataDashboard("|--  ShooterDown power: " , shooterDown.getPower());
        Logger.addDataDashboard("|--  ShooterUp velocity in radians: " , shooterUp.getVelocity(AngleUnit.RADIANS));
        Logger.addDataDashboard("|--  ShooterDown velocity in radians: " , shooterDown.getVelocity(AngleUnit.RADIANS));
        Logger.addData("Ticks:");
        Logger.addDataDashboard("|--  RightSide ticks: " , rightSide.getCurrentPosition());
        Logger.addDataDashboard("|--  LeftSide ticks: " , leftSide.getCurrentPosition());
        Logger.addDataDashboard("|--  Conveyor ticks: " , conveyor.getCurrentPosition());
        Logger.addDataDashboard("|--  collector ticks: " , collector.getCurrentPosition());
        Logger.addDataDashboard("|--  ShooterUp ticks: " , shooterUp.getCurrentPosition());
        Logger.addDataDashboard("|--  ShooterDown ticks: " , shooterDown.getCurrentPosition());
        Logger.addData("Info (usually variables):");
        Logger.addDataDashboard("|--  shooter step: " , shooterStep);
        Logger.addDataDashboard("|--  prevTime: " , prevTime);
        Logger.addDataDashboard("|--  ShooterUp target velocity in radians: " , targetVeloRadUp);
        Logger.addDataDashboard("|--  ShooterDown target velocity in radians: " , targetVeloRadDown);
        Logger.addData("Position:");
        Logger.addDataDashboard("|--  X: " , tracker.getPosition().X);
        Logger.addDataDashboard("|--  Y: " , tracker.getPosition().Y);
        Logger.addDataDashboard("|--  Z: " , tracker.getPosition().Z);
        Logger.addDataDashboard("|--  targetAngle: " , tracker.angleToTarget(BASKET_TARGET));
        Logger.addDataDashboard("|--  DistanceToTarget center: ", tracker.distanceToTarget(BASKET_TARGET));
        Logger.addDataDashboard("|--  currentAngle: " , -imu.getAngularOrientation().firstAngle);
        Logger.addData("Booleans: ");
        Logger.addDataDashboard("|-- insideShootingCircle: ", tracker.checkInsideCircle(BASKET_TARGET, SHOOTING_CIRCLE_RADIUS));
        Logger.addDataDashboard("|-- Inside no-man's land: ", tracker.checkInsideCircle(BASKET_TARGET, minimumDistanceToTarget));
        Logger.addDataDashboard("|-- shooterActivated: ", shooterActivated);
        Logger.update();
    }

    @Override
    public void runOpMode() {

        initHardware();
        Logger.setTelemetry(telemetry);

        waitForStart();
        while (opModeIsActive()) {

            globalPowerFactorControl();

            driveTrainControl();

            locator();

            main();

            conveyorControl();

            collectorControl();

            Logging();
        }
    }

}
