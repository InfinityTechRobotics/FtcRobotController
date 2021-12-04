package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="Test Arm")
public class TestArm extends OpMode {

    public DcMotor mShoulder = null;
    public DcMotor mElbow = null;

    public void init(){
        mShoulder = hardwareMap.get(DcMotor.class, "LiftShoulderPivot");
        mShoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mElbow = hardwareMap.get(DcMotor.class, "LiftElbowPivot");
        mElbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {

        double shoulderinput = gamepad1.left_stick_y;
        double elbowinput = gamepad1.right_stick_y;

        double shoulderpower = 0.6*shoulderinput;
        double elbowpower = 0.6*elbowinput;

        mShoulder.setPower(shoulderpower);
        mElbow.setPower(elbowpower);
    }
}
