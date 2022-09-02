package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.z3db0y.davidlib.DriveTrain;
import com.z3db0y.davidlib.LocationTracker;
import com.z3db0y.davidlib.Logger;
import com.z3db0y.davidlib.Motor;
import com.z3db0y.davidlib.Vector;

import java.util.HashMap;

@TeleOp(name = "Location Tracker Test", group = "FGC22")
public class LocationTrackerTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Motor leftMotor = new Motor(hardwareMap, "leftSide");
        Motor rightMotor = new Motor(hardwareMap, "rightSide");
        HashMap<DriveTrain.Location, Motor> motors = new HashMap<>();
        motors.put(DriveTrain.Location.BACKLEFT, leftMotor);
        motors.put(DriveTrain.Location.BACKRIGHT, rightMotor);
        DriveTrain driveTrain = new DriveTrain(DriveTrain.Type.TWOWHEELDRIVE, motors);
        LocationTracker tracker = new LocationTracker();
        LocationTracker.Parameters params = tracker.new Parameters();
        params.driveTrain = driveTrain;
        params.motorTicksPerRevolution = 28 * 60;
        params.robotWidth = 42;
        params.wheelRadius = 4.5;
        tracker.initialize(params);

        waitForStart();

        Logger.setTelemetry(telemetry);
        while (opModeIsActive()) {
            Vector pos = tracker.updatePosition();
            Logger.addData("X: " + pos.X);
            Logger.addData("Z: " + pos.Z);
            Logger.update();
        }
    }

}
