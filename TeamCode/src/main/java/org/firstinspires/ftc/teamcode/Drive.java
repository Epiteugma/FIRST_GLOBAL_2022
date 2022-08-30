package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.z3db0y.davidlib.
import com.z3db0y.davidlib.Motor;

@TeleOp(name = "TeleOpFGC", group = "FGC22")
public class Drive extends LinearOpMode {
    Motor leftSide;
    Motor rightSide;
    Motor conveyor;
    Motor collector;
    Motor shooterDown;
    Motor shooterUp;
    BNO055IMU imu;

    public void initHardware() {
        leftSide = new Motor(hardwareMap, "leftSide");
        rightSide = new Motor(hardwareMap, "rightSide");
        collector = new Motor(hardwareMap, "collector");
        conveyor = new Motor(hardwareMap, "conveyor");

        shooterDown = new Motor(hardwareMap, "shooterDown");
        shooterUp = new Motor(hardwareMap, "shooterUp");

        // imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        leftSide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Directions
        shooterUp.setDirection(Motor.Direction.REVERSE);
        shooterDown.setDirection(Motor.Direction.REVERSE);
        leftSide.setDirection(Motor.Direction.REVERSE);
        rightSide.setDirection(Motor.Direction.REVERSE);
    }

    double collectorPower = Configurable.collectorPower;
    double conveyorPower = Configurable.conveyorPower;
//    double shooterUpperPower = Configurable.shooterUpperPower;
//    double shooterLowerPower = Configurable.shooterLowerPower;
    double shooterStep = Configurable.shooterStep;
    double globalPowerFactor = 0.4;
    long prevTime = 0;
    double targetVeloUp = 0;
    double targetVeloDown = 0;

    private void controlShooter(double distanceFromTarget) {
        double massOfTheBall = Configurable.massOfTheBall;
        double gravity = Configurable.gravity;
        double angleToTarget = Configurable.angleToTarget;
        double verticalDistanceToTarget = Configurable.verticalDistanceToTarget;
        double targetAngleTan = Math.tan(Math.toRadians(angleToTarget));
        double minimDistanceToTarget = verticalDistanceToTarget / targetAngleTan;

        double velocity = Math.sqrt(gravity * Math.pow(distanceFromTarget, 2) * (1 + Math.pow(targetAngleTan, 2)) /
                2 * (distanceFromTarget * targetAngleTan - verticalDistanceToTarget));
        // D * targetAngleTan has to be bigger/equal(risky) than verticalDistanceToTarget

        targetVeloUp = velocity;
        if ( distanceFromTarget >= minimDistanceToTarget) {
            targetVeloDown = velocity;
        }
//            shooterUp.setVelocity(velocity);
//            shooterDown.setVelocity(velocity);
        else {
//            shooterUp.setVelocity(velocity);
            double multiplier = distanceFromTarget * 0.3;
            Logger.addData("Multiplier" + multiplier);
            targetVeloDown = velocity * multiplier;
//            shooterDown.setVelocity(targetVeloDown);
        }
    }

    private void shooterControl(){
        double shooterVeloStep = Configurable.shooterVeloStep;
        double distanceFromTarget = Configurable.distanceFromTarget;
        if (gamepad1.circle) {
            controlShooter(distanceFromTarget);
        }
        else if(gamepad1.triangle){
            targetVeloUp = targetVeloDown;
            targetVeloDown += shooterVeloStep;
        }
        shooterUp.setVelocity(targetVeloUp);
        shooterDown.setVelocity(targetVeloDown);
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
        Logger.addData("UpVelo" + shooterUp.getVelocity());
        Logger.addData("DownVelo" + shooterDown.getVelocity());
        double upVelo = shooterUp.getVelocity();
        double downVelo = shooterDown.getVelocity();
        if (Math.abs(upVelo - targetVeloUp) > shooterStep && Math.abs(downVelo - targetVeloDown) > shooterStep) {
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
        Logger.addData("Ticks:");
        Logger.addData("|--  RightSide ticks: " + rightSide.getCurrentPosition());
        Logger.addData("|--  LeftSide ticks: " + leftSide.getCurrentPosition());
        Logger.addData("|--  Conveyor ticks: " + conveyor.getTargetPosition());
        Logger.addData("|--  collector ticks: " + collector.getCurrentPosition());
        Logger.addData("Info (usually variables):");
        Logger.addData("|--  shooter step: " + shooterStep);
        Logger.addData("|--  prevTime: " + prevTime);
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
