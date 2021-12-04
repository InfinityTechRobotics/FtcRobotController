package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {
    public DcMotor mShoulder = null;
    public DcMotor mElbow = null;

    public void init(HardwareMap hardwareMap) {
        /*
            This assumes two motors in the config file are named LiftShoulderPivot and LiftElbowPivot
         */
        
        mShoulder = hardwareMap.get(DcMotor.class, "LiftShoulderPivot");
        mShoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mElbow = hardwareMap.get(DcMotor.class, "LiftElbowPivot");
        mElbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


}
