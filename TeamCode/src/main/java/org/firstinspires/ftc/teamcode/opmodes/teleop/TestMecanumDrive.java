package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@TeleOp(name="Test Mecanum Drive")
public class TestMecanumDrive extends OpMode {

    // Declare Hardware object
    Robot myRobot;

    // Constructor
    public TestMecanumDrive() {

        // Instantiate Objects
        myRobot = new Robot();

    }

    @Override
    public void init() {

        // Initialize Hardware
        myRobot.init(hardwareMap);

    }

    @Override
    public void loop() {

        // read inputs from gamepad
        double Y1 = gamepad1.left_stick_y;
        double Y2 = gamepad1.right_stick_y;
        double X1 = gamepad1.left_stick_x;
        double X2 = gamepad1.right_stick_x;

        // calculate drive vector from gamepad inputs
        //
        double forward = -(Y1+Y2)/2;
        //
        double right = X2;
        //
        double clockwise = -(Y1-Y2)/2;

        // Send drive vector to the robot object
        myRobot.vector(
                forward,
                right,
                clockwise);

        telemetry.addData("Y1",Y1);
        telemetry.addData("Y2", Y2);
        telemetry.addData("X1", X1);
        telemetry.addData("X2", X2);
    }

}
