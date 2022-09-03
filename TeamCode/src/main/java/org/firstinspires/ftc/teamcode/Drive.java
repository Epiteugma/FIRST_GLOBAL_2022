package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.z3db0y.davidlib.Logger;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

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
        leftSide.setDirection(DcMotorEx.Direction.FORWARD);
        rightSide.setDirection(DcMotorEx.Direction.FORWARD);
    }

    double collectorPower = Configurable.collectorPower;
    double conveyorPower = Configurable.conveyorPower;
    double shooterStep = Configurable.shooterStep;
    double shooterMarginOfError = Configurable.shooterMarginOfError;
    double globalPowerFactor = 0.7;
    long prevTime = 0;
    double targetVeloRadUp = 0;
    double targetVeloRadDown = 0;
    PIDFCoefficients shooterUpCoeffs = Configurable.shooterUpCoeffs;
    PIDFCoefficients shooterDownCoeffs = Configurable.shooterDownCoeffs;
    ElapsedTime PIDFtimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    double lastShooterUpError = 0;
    double lastShooterDownError = 0;

    public void shooterCalculator(DcMotorEx motor, double horizontalDistanceToTarget) {
        double minShooterVelo = Configurable.minShooterVeloRads;
        double maxShooterVelo = Configurable.maxShooterVeloRads;
        double shooterGearRatio = Configurable.shooterGearRatio;
        double shooterWheelRadiusStart = Configurable.shooterWheelRadiusStart;
        double shooterWheelMaxExpansion = Configurable.shooterWheelMaxExpansion;
        double verticalDistanceToTarget = Configurable.verticalDistanceToTarget;
        double angleToTarget= imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle + 90; // control hub is X degrees off the way it is placed
        double angleToTargetTan = Math.tan(Math.toRadians(angleToTarget));
        double minimumDistanceToTarget = horizontalDistanceToTarget / angleToTargetTan;
        double sqrdHorizontalDistanceToTarget = Math.pow(horizontalDistanceToTarget, 2);
        double sqrdTargetAngleTan = Math.pow(angleToTargetTan, 2);
        Acceleration gravity = imu.getGravity();
        double gravityMagnitude = Math.sqrt(Math.pow(gravity.xAccel, 2) + Math.pow(gravity.yAccel, 2) + Math.pow(gravity.zAccel, 2));

        Logger.addData("Angle to Target" + angleToTarget);

        double targetVelo = Math.sqrt(gravityMagnitude * sqrdHorizontalDistanceToTarget * (1 + sqrdTargetAngleTan) /
                (2 * (horizontalDistanceToTarget * angleToTargetTan - verticalDistanceToTarget)));
        double currentVelo = motor.getVelocity(AngleUnit.RADIANS);
        double shooterWheelRadius = shooterWheelRadiusStart + (shooterWheelMaxExpansion * currentVelo/ maxShooterVelo);
        double targetVeloRad = targetVelo / shooterGearRatio / shooterWheelRadius;

        Logger.addData("targetVelo / shooterGearRatio / shooterWheelRadius: " + targetVelo + "/" + shooterGearRatio + "/" + shooterWheelRadius + "=" + targetVeloRad);

        targetVeloRadUp = targetVeloRad;
        // D * targetAngleTan has to be bigger/equal(risky) than verticalDistanceToTarget
        if ( horizontalDistanceToTarget < minimumDistanceToTarget) {
            double radsAdditionRange = Configurable.radsAdditionRange;
            double radsDivider = minimumDistanceToTarget / radsAdditionRange;
            double radIncrease = horizontalDistanceToTarget / radsDivider;
            targetVeloRadDown = targetVeloRad + radIncrease;
        }
        else {
            targetVeloRadDown = targetVeloRad;
        }
    }

    private void shooterControl(){
        PIDFtimer.reset();
        double shooterStep = Configurable.shooterStep;
        double distanceFromTarget = Configurable.horizontalDistanceToTarget;
        shooterCalculator(shooterDown, distanceFromTarget);
        shooterCalculator(shooterUp, distanceFromTarget);
        if (gamepad1.circle) {
            shooterCalculator(shooterDown, distanceFromTarget);
            shooterCalculator(shooterUp, distanceFromTarget);
        }
        else if(gamepad1.triangle){
            targetVeloRadUp += shooterStep;
            targetVeloRadDown += shooterStep;
        }
        else if(gamepad1.cross){
            targetVeloRadUp -= shooterStep;
            targetVeloRadDown -= shooterStep;
        }
        double shooterUpVelo = shooterUp.getVelocity(AngleUnit.RADIANS);
        double shooterDownVelo = shooterDown.getVelocity(AngleUnit.RADIANS);
        double shooterUpError = targetVeloRadUp - shooterUpVelo;
        double shooterDownError = targetVeloRadDown - shooterDownVelo;
        double shooterUpIntegral = shooterUpError * PIDFtimer.time();
        double shooterDownIntegral = shooterDownError * PIDFtimer.time();
        double shooterUpDeltaError = shooterUpError - lastShooterUpError;
        double shooterDownDeltaError = shooterDownError - lastShooterDownError;
        double shooterUpDerivative = shooterUpDeltaError / PIDFtimer.time();
        double shooterDownDerivative = shooterDownDeltaError / PIDFtimer.time();
        shooterUp.setVelocityPIDFCoefficients(shooterUpError * shooterUpCoeffs.p, shooterUpIntegral * shooterUpCoeffs.i, shooterUpDerivative * shooterUpCoeffs.d, shooterUpVelo * shooterUpCoeffs.f);
        shooterDown.setVelocityPIDFCoefficients(shooterDownError * shooterDownCoeffs.p, shooterDownIntegral * shooterDownCoeffs.i, shooterDownDerivative * shooterDownCoeffs.d, shooterDownVelo * shooterDownCoeffs.f);

        Logger.addDataDashboard("shooterDown velocity", shooterDownVelo);
        Logger.addDataDashboard("shooterUp velocity", shooterUpVelo);
        Logger.addDataDashboard("shooterDown target velocity", targetVeloRadDown);
        Logger.addDataDashboard("shooterUp target velocity", targetVeloRadUp);
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
        double upVelo = Math.abs(shooterUp.getVelocity(AngleUnit.RADIANS));
        double downVelo = Math.abs(shooterDown.getVelocity(AngleUnit.RADIANS));
        Logger.addData(upVelo);
        Logger.addData(downVelo);
        if (Math.abs(upVelo - targetVeloRadUp) < shooterMarginOfError && Math.abs(downVelo - targetVeloRadDown) < shooterMarginOfError && upVelo > 0 && downVelo > 0) {
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

            globalPowerFactorControl();

            driveTrainControl();

            shooterControl();

            conveyorControl();

            collectorControl();

            Logging();
        }
    }

}
