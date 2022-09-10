package com.z3db0y.davidlib;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;

public class Motor extends DcMotorImplEx {

    public Motor(DcMotorImplEx motor) {
        super(motor.getController(), motor.getPortNumber());
    }

}
