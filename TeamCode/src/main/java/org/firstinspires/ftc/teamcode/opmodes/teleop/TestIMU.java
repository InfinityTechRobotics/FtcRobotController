package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;

/**
 * tests `robot.getHeading()`
 * just rub this opmode and spin the robot to see what happens
 */

@TeleOp(name="Test IMU")
public class TestIMU extends OpMode {

    private Robot robot;

    @Override
    public void init() {
        robot = new Robot();
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {

        telemetry.addData("Heading: ", robot.getHeading());
        telemetry.update();

    }
}
