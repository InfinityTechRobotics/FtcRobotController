package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="Test Mecanum Drive")
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
        double X2 = gamepad2.right_stick_x;

        // calculate drive vector from gamepad inputs
        // push both joysticks forward to go forward
        double forward = -(Y1+Y2)/2;
        // push joystick2 to the right to strafe right
        double right = X2;
        // push joystick1 forward and pull joystick2 backward
        double clockwise = -(Y1-Y2)/2;
        // to rotate clockwise

        // Send drive vector to the robot object
        myRobot.vector(
                forward,
                right,
                clockwise);
    }

}
