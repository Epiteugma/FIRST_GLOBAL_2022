package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.z3db0y.davidlib.Logger;
import com.z3db0y.davidlib.Motor;

@Config
@TeleOp(name = "VeloTest", group = "Testing")
public class VeloTest extends LinearOpMode {
    public static double targetVelo = 5; // m/s, configurable

    double wheelRadius = 2.54; // CM
    double maxExpansion = 0.25; // CM
    double maxVelo = 2450;

    @Override
    public void runOpMode() {
        Motor motor = new Motor(hardwareMap.get(DcMotorImplEx.class, "shooterUp"));
        Logger.setTelemetry(telemetry);
        waitForStart();

//        double start = System.currentTimeMillis();
//        while(System.currentTimeMillis() < start + 5000) {
//            motor.setPower(1);
//            double velo = motor.getVelocity();
//            if(velo > maxVelo) maxVelo = velo;
//        }
//        motor.setPower(0);

        while(opModeIsActive()) {
            int tpr = 28;
            double calculatedRadius = wheelRadius + maxExpansion * (motor.getVelocity() / maxVelo);
            double targetAngularVelo = targetVelo * 100 / (2 * Math.PI * calculatedRadius) * tpr;
            motor.setVelocity(targetAngularVelo);
            Logger.addDataDashboard("Wheel radius", calculatedRadius);
            Logger.addDataDashboard("Max velocity", maxVelo);
            double velo = motor.getVelocity() / tpr * 2 * Math.PI * calculatedRadius;
            Logger.addDataDashboard("Velocity (m/s)", velo / 100);
            Logger.update();
        }
    }
}
