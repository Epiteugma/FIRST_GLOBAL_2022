package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.z3db0y.davidlib.Logger;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "TeleOpFGC", group = "FGC22")
public class Drive extends LinearOpMode {
    DcMotorEx leftSide;
    DcMotorEx rightSide;
    DcMotorEx conveyor;
    DcMotorEx collector;
    DcMotorEx shooterDown;
    DcMotorEx shooterUp;
    BNO055IMU imu;

    public void initHardware() {
        leftSide = hardwareMap.get(DcMotorEx.class, "leftSide");
        rightSide = hardwareMap.get(DcMotorEx.class, "rightSide");
        collector = hardwareMap.get(DcMotorEx.class, "collector");
        conveyor = hardwareMap.get(DcMotorEx.class, "conveyor");

        shooterDown = hardwareMap.get(DcMotorEx.class, "shooterDown");
        shooterUp = hardwareMap.get(DcMotorEx.class, "shooterUp");

        // imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        leftSide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightSide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Directions
        shooterUp.setDirection(DcMotorEx.Direction.REVERSE);
        shooterDown.setDirection(DcMotorEx.Direction.REVERSE);
        leftSide.setDirection(DcMotorEx.Direction.REVERSE);
        rightSide.setDirection(DcMotorEx.Direction.REVERSE);
    }

    double collectorPower = Configurable.collectorPower;
    double conveyorPower = Configurable.conveyorPower;
    double shooterStep = Configurable.shooterStep;
    double globalPowerFactor = 0.4;
    long prevTime = 0;
    double targetVeloRadUp = 0;
    double targetVeloRadDown = 0;

    public void controlShooter(double horizontalDistanceToTarget) {
        double massOfTheBall = Configurable.massOfTheBall;
        double shooterGearRatio = Configurable.shooterGearRatio;
        double shooterWheelRadius = Configurable.shooterWheelRadius;
        double gravity = Configurable.gravity;
        double angleToTarget = Configurable.angleToTarget;
        double verticalDistanceToTarget = Configurable.verticalDistanceToTarget;
        double targetAngleTan = Math.tan(Math.toRadians(angleToTarget));
        double minimumDistanceToTarget = horizontalDistanceToTarget / targetAngleTan;
//        double shooterTicksPerRev = Configurable.shooterTicksPerRev;
        double sqrdHorizontalDistanceToTarget = Math.pow(horizontalDistanceToTarget, 2);
        double sqrdTargetAngleTan = Math.pow(targetAngleTan, 2);

        double targetVelo = Math.sqrt(gravity * sqrdHorizontalDistanceToTarget * (1 + sqrdTargetAngleTan) /
                (2 * (horizontalDistanceToTarget * targetAngleTan - verticalDistanceToTarget)));
        // D * targetAngleTan has to be bigger/equal(risky) than verticalDistanceToTarget
        double targetVeloRad = targetVelo / shooterGearRatio / shooterWheelRadius;

        Logger.addData("Calculated velocity in radians" + targetVeloRad + "with shooterWheelRadius" + shooterWheelRadius);

        targetVeloRadUp = targetVeloRad;
        if ( horizontalDistanceToTarget <= minimumDistanceToTarget) {
            double multiplier = horizontalDistanceToTarget * 0.001; // need a formula for needed height correction compensating in shooterUpVeloRad
            Logger.addData("Multiplier" + multiplier);
            targetVeloRadDown = targetVeloRad * multiplier;
        }
        else {
            targetVeloRadDown = targetVeloRad;
        }
    }

    private void shooterControl(){
        double shooterVeloRadStep = Configurable.shooterVeloRadStep;
        double distanceFromTarget = Configurable.horizontalDistanceToTarget;
//        if (gamepad1.circle) {
            controlShooter(distanceFromTarget);
//        }
        if(gamepad1.triangle){
            targetVeloRadUp = targetVeloRadDown;
            targetVeloRadDown += shooterVeloRadStep;
        }
//        shooterUp.setPower(shooterUpPower);
//        shooterDown.setPower(shooterDownPower);
        shooterUp.setVelocity(targetVeloRadUp, AngleUnit.RADIANS);
        shooterDown.setVelocity(targetVeloRadDown, AngleUnit.RADIANS);
    }

    private void globalPowerFactorControl() {
        if (gamepad1.left_bumper && globalPowerFactor - 0.1 > 0 && System.currentTimeMillis() >= prevTime + 500) {
            prevTime = System.currentTimeMillis();
            globalPowerFactor -= 0.1;
        } else if (gamepad1.right_bumper && globalPowerFactor + 0.1 < 0.7 && System.currentTimeMillis() >= prevTime + 500) {
            prevTime = System.currentTimeMillis();
            globalPowerFactor += 0.1;
        }

        if (gamepad1.triangle && System.currentTimeMillis() >= prevTime + 500) {
            prevTime = System.currentTimeMillis();
            if (Math.abs(globalPowerFactor - 0.3) < Math.abs(globalPowerFactor - 0.6)) {
                globalPowerFactor = 0.6;
            } else {
                globalPowerFactor = 0.3;
            }
        }
        if (gamepad1.cross && System.currentTimeMillis() >= prevTime + 500) {
            prevTime = System.currentTimeMillis();
            globalPowerFactor = 1;
        }
    }

    private void driveTrainControl(){
        double yleft = gamepad1.left_stick_y;
        double yright = gamepad1.right_stick_y;
        leftSide.setPower(yleft * globalPowerFactor);
        rightSide.setPower(yright * globalPowerFactor);
    }

    private void conveyorControl(){
        double upVelo = Math.abs(shooterUp.getVelocity());
        double downVelo = Math.abs(shooterDown.getVelocity());
        Logger.addData(upVelo);
        Logger.addData(downVelo);
        if (Math.abs(upVelo - targetVeloRadUp) < shooterStep && Math.abs(downVelo - targetVeloRadDown) < shooterStep && upVelo > 0 && downVelo > 0) {
            conveyor.setPower(conveyorPower);
        }
        else {
            conveyor.setPower(0);
        }
    }

    private void collectorControl() {
        collector.setPower(collectorPower);
    }

    private void communication(){
        if(gamepad1.right_trigger > 0) {
            gamepad2.rumble(gamepad1.right_trigger, gamepad1.right_trigger, 1);
        }
    }

    private void Logging() {
        Logger.addData("Powers:");
        Logger.addData("|--  GlobalPowerFactor: " + globalPowerFactor);
        Logger.addData("|--  Right Side power: " + rightSide.getPower());
        Logger.addData("|--  Left Side power: " + leftSide.getPower());
        Logger.addData("|--  Conveyor power: " + conveyor.getPower());
        Logger.addData("|--  Collector power: " + collector.getPower());
        Logger.addData("|--  ShooterUp power: " + shooterUp.getPower());
        Logger.addData("|--  ShooterDown power: " + shooterDown.getPower());
        Logger.addData("|--  ShooterUp velocity in radians: " + shooterUp.getVelocity(AngleUnit.RADIANS));
        Logger.addData("|--  ShooterDown velocity in radians: " + shooterDown.getVelocity(AngleUnit.RADIANS));
        Logger.addData("Ticks:");
        Logger.addData("|--  RightSide ticks: " + rightSide.getCurrentPosition());
        Logger.addData("|--  LeftSide ticks: " + leftSide.getCurrentPosition());
        Logger.addData("|--  Conveyor ticks: " + conveyor.getTargetPosition());
        Logger.addData("|--  collector ticks: " + collector.getCurrentPosition());
        Logger.addData("|--  ShooterUp ticks: " + shooterUp.getCurrentPosition());
        Logger.addData("|--  ShooterDown ticks: " + shooterDown.getCurrentPosition());
        Logger.addData("Info (usually variables):");
        Logger.addData("|--  shooter step: " + shooterStep);
        Logger.addData("|--  prevTime: " + prevTime);
        Logger.addData("|--  ShooterUp target velocity in radians: " + targetVeloRadUp);
        Logger.addData("|--  ShooterDown target velocity in radians: " + targetVeloRadDown);
        Logger.update();
    }

    @Override
    public void runOpMode() {


        initHardware();
        Logger.setTelemetry(telemetry);

        waitForStart();
        while (opModeIsActive()) {

            communication();

            driveTrainControl();

            globalPowerFactorControl();

            shooterControl();

            conveyorControl();

            collectorControl();

            Logging();
        }
    }

}
