package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.Arm;

@TeleOp(name="Test Arm")
public class TestArm extends OpMode {

    public Arm arm;

    @Override
    public void init() {
        arm.init(hardwareMap);
    }

    @Override
    public void loop() {

        double shoulderinput = gamepad1.left_stick_y;
        double elbowinput = gamepad1.right_stick_y;

        double shoulderpower = 0.1*shoulderinput;
        double elbowpower = 0.1*elbowinput;

        arm.mShoulder.setPower(shoulderpower);
        arm.mElbow.setPower(elbowpower);
    }
}
