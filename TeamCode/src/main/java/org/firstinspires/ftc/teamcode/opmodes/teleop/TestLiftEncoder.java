package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@TeleOp(name="Test Lift Encoder")
public class TestLiftEncoder extends OpMode {

    private Robot robot = null;

    @Override
    public void init() {
        robot = new Robot();
        robot.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {

        /* Slider */
        double sliderUp = gamepad2.right_trigger;
        double sliderDown = gamepad2.left_trigger;
        robot.setSlidePower(sliderUp - sliderDown);

        telemetry.addData("Lift Encoder: ", robot.getLiftEnc());
        telemetry.update();

    }
}
