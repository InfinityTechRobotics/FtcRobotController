package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@TeleOp(name="Test Drive")
public class TestDrive extends OpMode {

    // Declare Hardware object
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftRear;
    public DcMotor rightRear;

    // Constructor
    public TestDrive() {

        // Instantiate Objects


    }

    @Override
    public void init() {

        leftFront = hardwareMap.get(DcMotor.class,"lf");
        rightFront = hardwareMap.get(DcMotor.class,"rf");
        leftRear = hardwareMap.get(DcMotor.class,"lr");
        rightRear = hardwareMap.get(DcMotor.class,"rr");
    }

    @Override
    public void loop() {

        /* Drivetrain */
        // read inputs from gamepad
        double Y1 = Math.pow(gamepad1.left_stick_y, 3);
        double Y2 = Math.pow(gamepad1.right_stick_y, 3);
        double X = Math.pow(gamepad1.right_stick_x, 3);
        // calculate drive vector from gamepad input
        // forward is the average of the left and right sticks in the y direction
        // range is from -1.0 to +1.0
        double forward = -(Y1+Y2)/2;
        // right, aka strafe, is simply the x value from the right stick
        // range is from -1.0 to +1.0
        double right = X;
        // clockwise, aka twist, is the difference between the left and right sticks in the y direction
        // range is from -1.0 to +1.0
        double clockwise = -(Y1-Y2)/2;
        // Add telemetry for the raw inputs
        telemetry.addData("Drive: ", "(%.2f)", forward);
        telemetry.addData("Strafe: ", "(%.2f)", right);
        telemetry.addData("Twist: ","(%.2f)",clockwise);
        // Set a scale factor to reduce the sensitivity of the forward and twist motions
        double forwardscalefactor = 0.5;
        double rightscalefactor = 1.0;
        double clockwisescalefactor = 0.5;
        // Send drive vector to the robot object
        vector(forward*forwardscalefactor, right*rightscalefactor, clockwise*clockwisescalefactor);

    }

    public void vector(double drive, double strafe, double twist) {

        // apply calculated speeds to the drive motors
        leftFront.setPower(drive + strafe + twist);
        rightFront.setPower(drive - strafe - twist);
        leftRear.setPower(drive - strafe + twist);
        rightRear.setPower(drive + strafe - twist);

    }
}
