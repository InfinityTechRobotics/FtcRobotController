package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {

    // Declare Actuators
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    private DcMotor intake = null;
    private DcMotor lift = null;
    private Servo deliver = null;

    private DcMotor carousel = null;

    // Initialize the Hardware
    // THIS FUNCTION MUST BE CALLED
    public void init(HardwareMap hardwareMap) {

        // Initialize drive motors
        frontLeft = hardwareMap.get(DcMotor.class, "lf");
        frontRight = hardwareMap.get(DcMotor.class, "rf");
        backLeft = hardwareMap.get(DcMotor.class, "lf");
        backRight = hardwareMap.get(DcMotor.class, "rr");

        // TODO might need to fix this . . .
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO Initialize intake motor
        intake = hardwareMap.get(DcMotor.class, "intake");

        // TODO initialize lift motor
        lift = hardwareMap.get(DcMotor.class,"lift");

        // TODO initialize deliver servo
        deliver = hardwareMap.get(Servo.class, "deliver");

        // TODO initialize carousel motor
        carousel = hardwareMap.get(DcMotor.class, "carousel");
    }

    public void vector(double drive, double strafe, double twist) {

        double[] speeds = new double[]{
                (drive + strafe + twist), // front left
                (drive - strafe - twist), // front right
                (drive - strafe + twist), // back left
                (drive + strafe - twist) // back right
        };

        // apply calculated speeds to the drive motors
        frontLeft.setPower(speeds[0]);
        frontRight.setPower(speeds[1]);
        backLeft.setPower(speeds[2]);
        backRight.setPower(speeds[3]);

    }

    public void startIntake() {
        // TODO may need to correct the direction of the intake motor
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setPower(0.25);
    }

    public void stopIntake() {
        intake.setPower(0.0);
    }

    public void reverseIntake() {
        // TODO this needs to be the opposite of the startIntake direction
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setPower(0.25);
    }

    public void startLift() {
        // TODO need to verify this direction is correct
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setPower(0.5);
    }

    public void stopLift() {
        lift.setPower(0.0);
    }

    public void reverseLift() {
        // TODO need to verify this direction is correct.  Must be the opposite of startLift
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setPower(0.5);
    }

    public void startCarousel() {
        carousel.setDirection(DcMotorSimple.Direction.FORWARD);
        carousel.setPower(0.25);
    }

    public void stopCarousel() {
        carousel.setPower(0.0);
    }
}
