package org.firstinspires.ftc.teamcode.hardware;

import android.os.Build;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.function.BooleanSupplier;

public class Robot {

    // Declare Constants
    public static final double DUMP_POSITION = -1.0d; // TODO check this . . .
    public static final double UNDUMP_POSITION = +1.0d; // TODO check this . . .
    public static final double INTAKE_POWER = 0.8d;
    public static final double CAROUSEL_POWER = 0.25;
    public static final double LIFT_POWER_SCALE_FACTOR = 1.0;

    // Declare Actuators
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor intake = null;
    private DcMotor lift = null;
    private Servo deliver = null;
    private DcMotor carousel = null;
    private IMU imu = null;

    // Declare global variables
    public long liftPosition = 0;
    public boolean isMoving = false;

    // Class to represent mecanum motor speed
    public class MecanumMotorSpeed {
        double[] speeds;
        double frontLeft, frontRight, backLeft, backRight;
        MecanumMotorSpeed(double frontLeft, double frontRight, double backLeft, double backRight) {
            // Load Speeds
            speeds = new double[]{frontLeft, frontRight, backLeft, backRight};
            // Find Largest Speed
            double max = Math.abs(speeds[0]);
            for(double speed : speeds) if(Math.abs(speed) > max) max = Math.abs(speed);
            // Reduce All Speeds If Max Speed Is Outside Allowed Range of [-1,1]
            if(max > 1d) for(int i = 0 ; i < speeds.length ; ++i) speeds[i] /= max;
            this.frontLeft = speeds[0];
            this.frontRight = speeds[1];
            this.backLeft = speeds[2];
            this.backRight = speeds[3];
        }
    }

    // Initialize the Hardware
    // THIS FUNCTION MUST BE CALLED
    public void init(HardwareMap hardwareMap) {

        // Initialize drive motors
        frontLeft = hardwareMap.get(DcMotor.class, "lf");
        frontRight = hardwareMap.get(DcMotor.class, "rf");
        backLeft = hardwareMap.get(DcMotor.class, "lr");
        backRight = hardwareMap.get(DcMotor.class, "rr");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize intake motor
        intake = hardwareMap.get(DcMotor.class, "intake");

        // Initialize lift motor
        lift = hardwareMap.get(DcMotor.class,"lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize deliver servo
        deliver = hardwareMap.get(Servo.class, "deliver");
        // Set the initial position as 0.0d
        //deliver.setPosition(0.0d);
        // deliver.scaleRange(0.0d, 0.5d);

        // Initialize carousel motor
        carousel = hardwareMap.get(DcMotor.class, "carousel");

        imu = new IMU();
        try {
            imu.initIMU(hardwareMap);
        } catch(Exception e) {
            // TODO log this somehow . . .
        }

    }

    public void vector(double drive, double strafe, double twist) {

        MecanumMotorSpeed speed = new MecanumMotorSpeed(
                (drive + strafe + twist), // front left
                (drive - strafe - twist), // front right
                (drive - strafe + twist), // back left
                (drive + strafe - twist) // back right
        );

        // apply calculated speeds to the drive motors
        frontLeft.setPower(speed.frontLeft);
        frontRight.setPower(speed.frontRight);
        backLeft.setPower(speed.backLeft);
        backRight.setPower(speed.backRight);

    }

    public void collect() {
        intake.setPower(INTAKE_POWER);
    }

    public void eject() {
        intake.setPower(-INTAKE_POWER);
    }

    public void setSlidePower(double pwr) {
        lift.setPower(LIFT_POWER_SCALE_FACTOR*pwr);
    }

    public void dump() {
        deliver.setPosition(DUMP_POSITION);
    }

    public void unDump() {
        deliver.setPosition(UNDUMP_POSITION);
    }

    public void setWOFPower(double pwr) {
        carousel.setPower(pwr);
    }

    public void stopIntake() {
        intake.setPower(0.0);
    }

    public void stopLift() {
        lift.setPower(0.0);
    }

    public void stopCarousel() {
        carousel.setPower(0.0);
    }

    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     *
     *  Note: removed Telemetry tm and BooleanSupplier omActive from list of parameters because I have no idea what these need as inputs
     */
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newFrontLeftTarget;
        int     newFrontRightTarget;
        int     newBackLeftTarget;
        int     newBackRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
            if (true) {

                // Determine new target position, and pass to motor controller
                moveCounts = (int)(distance * Constants.COUNTS_PER_INCH);
                newFrontRightTarget = frontRight.getCurrentPosition() + moveCounts;
                newFrontLeftTarget = frontLeft.getCurrentPosition() + moveCounts;
                newBackRightTarget = backRight.getCurrentPosition() + moveCounts;
                newBackLeftTarget = backLeft.getCurrentPosition() + moveCounts;

                // Set Target and Turn On RUN_TO_POSITION
                frontRight.setTargetPosition(newFrontLeftTarget);
                frontLeft.setTargetPosition(newFrontRightTarget);
                backLeft.setTargetPosition(newBackLeftTarget);
                backRight.setTargetPosition(newBackRightTarget);

                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // start motion.
                speed = Range.clip(Math.abs(speed), 0.0, 1.0);
                frontRight.setPower(speed);
                frontLeft.setPower(speed);
                backLeft.setPower(speed);
                backRight.setPower(speed);
                isMoving = true;

                // keep looping while we are still active, and BOTH motors are running.
                while ((frontLeft.isBusy() && frontRight.isBusy()
                                && backRight.isBusy() && backLeft.isBusy())) {

                    // adjust relative speed based on heading error.
                    error = getError(angle);
                    steer = getSteer(error, Constants.kP);

                    // if driving in reverse, the motor correction also needs to be reversed
                    if (distance < 0)
                        steer *= -1.0;

                    leftSpeed = speed - steer;
                    rightSpeed = speed + steer;

                    // Normalize speeds if either one exceeds +/- 1.0;
                    max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                    if (max > 1.0)
                    {
                        leftSpeed /= max;
                        rightSpeed /= max;
                    }

                    frontRight.setPower(speed);
                    frontLeft.setPower(speed);
                    backLeft.setPower(speed);
                    backRight.setPower(speed);

                }

                // Stop all motion;
                frontRight.setPower(0);
                frontLeft.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                isMoving = false;

                // Turn off RUN_TO_POSITION
                frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }

    public void Drive ( double speed,
                            double distance) {

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        // Ensure that the opmode is still active
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
            if (true) {

                // Determine new target position, and pass to motor controller
                moveCounts = (int) (distance * Constants.COUNTS_PER_INCH);
                newFrontRightTarget = frontRight.getCurrentPosition() + moveCounts;
                newFrontLeftTarget = frontLeft.getCurrentPosition() + moveCounts;
                newBackRightTarget = backRight.getCurrentPosition() + moveCounts;
                newBackLeftTarget = backLeft.getCurrentPosition() + moveCounts;

                // Set Target and Turn On RUN_TO_POSITION
                frontRight.setTargetPosition(newFrontRightTarget);
                frontLeft.setTargetPosition(newFrontLeftTarget);
                backLeft.setTargetPosition(newBackLeftTarget);
                backRight.setTargetPosition(newBackRightTarget);

                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // start motion.
                speed = Range.clip(Math.abs(speed), 0.0, 1.0);
                frontRight.setPower(speed);
                frontLeft.setPower(speed);
                backLeft.setPower(speed);
                backRight.setPower(speed);
                isMoving = true;

                // keep looping while we are still active, and BOTH motors are running.
                while ((frontLeft.isBusy() && frontRight.isBusy()
                        && backRight.isBusy() && backLeft.isBusy())) {

                    frontRight.setPower(speed);
                    frontLeft.setPower(speed);
                    backLeft.setPower(speed);
                    backRight.setPower(speed);

                }

                // Stop all motion;
                frontRight.setPower(0);
                frontLeft.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                isMoving = false;

                // Turn off RUN_TO_POSITION
                frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }

    public void Rotate ( double speed,
                        double angle) {
        int     moveCounts;
        int     newFrontRightTarget;
        int     newFrontLeftTarget;
        int     newBackRightTarget;
        int     newBackLeftTarget;
        double  leftSpeed;
        double  rightSpeed;
        double  startHeading;
        double  targetHeading;
        double  currentHeading;
        boolean clockwise;
        // set a range for error in the steering angle
        double  errorRange = 5.0;

        // Ensure that the opmode is still active
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
            if (true) {

                // Determine if turning clockwise or counterclockwise
                if (angle > 0) {
                    clockwise = true;
                } else {
                    clockwise = false;
                }

                // set leftSpeed and rightSpeed based on whether angle is positive or negative
                if (clockwise) {
                    // Positive angle means rotate clockwise, leftSpeed will be positive, rightSpeed will be negative
                    leftSpeed = Math.abs(speed);
                    rightSpeed = -Math.abs(speed);
                } else {
                    leftSpeed = -Math.abs(speed);
                    rightSpeed = Math.abs(speed);
                }

                // Determine new target position, and pass to motor controller
                moveCounts = (int)(angle * Constants.COUNTS_PER_DEGREE);
                if (clockwise) {
                    // left will be additive, right will be subtractive
                    // corrected directions based on observing robot
                    // newFrontRightTarget = frontRight.getCurrentPosition() - moveCounts;
                    newFrontLeftTarget = frontLeft.getCurrentPosition() + moveCounts;
                    // newBackRightTarget = backRight.getCurrentPosition() + moveCounts;
                    // newBackLeftTarget = backLeft.getCurrentPosition() + moveCounts;
                } else {
                    // right will be additive, left will be subtractive
                    // corrected directions based on observing robot
                    // newFrontRightTarget = frontRight.getCurrentPosition() + moveCounts;
                    newFrontLeftTarget = frontLeft.getCurrentPosition() - moveCounts;
                    // newBackRightTarget = backRight.getCurrentPosition() - moveCounts;
                    // newBackLeftTarget = backLeft.getCurrentPosition() - moveCounts;
                }

                // Set Target and Turn On RUN_TO_POSITION
                // frontRight.setTargetPosition(newFrontLeftTarget);
                frontLeft.setTargetPosition(newFrontLeftTarget);
                // backLeft.setTargetPosition(newBackLeftTarget);
                // backRight.setTargetPosition(newBackRightTarget);

                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // start motion.
                // speed = Range.clip(Math.abs(speed), 0.0, 1.0);
                frontRight.setPower(rightSpeed);
                frontLeft.setPower(leftSpeed);
                backLeft.setPower(leftSpeed);
                backRight.setPower(rightSpeed);
                isMoving = true;

                // keep looping while we are still active, and BOTH motors are running.
                while (frontLeft.isBusy()) {
                    isMoving = true;
                }

                // Stop all motion;
                frontRight.setPower(0);
                frontLeft.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                isMoving = false;

            }
        }
    }

    public void Strafe ( double speed,
                        double distance) {

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        // Ensure that the opmode is still active
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
            if (true) {

                // Determine new target position, and pass to motor controller
                moveCounts = (int) (distance * Constants.COUNTS_PER_INCH * Constants.STRAFE_SLIP_FACTOR);
                //newFrontRightTarget = frontRight.getCurrentPosition() + moveCounts;
                newFrontLeftTarget = frontLeft.getCurrentPosition() + moveCounts;
                // newBackRightTarget = backRight.getCurrentPosition() + moveCounts;
                // newBackLeftTarget = backLeft.getCurrentPosition() + moveCounts;

                // Set Target and Turn On RUN_TO_POSITION
                //frontRight.setTargetPosition(newFrontRightTarget);
                frontLeft.setTargetPosition(newFrontLeftTarget);
                // backLeft.setTargetPosition(newBackLeftTarget);
                // backRight.setTargetPosition(newBackRightTarget);

                frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (distance > 0) {
                    // Direction = right
                    speed = Range.clip(Math.abs(speed), 0.0, 1.0);
                    frontLeft.setPower(speed);
                    frontRight.setPower(-speed);
                    backLeft.setPower(-speed);
                    backRight.setPower(+speed);
                } else {
                    // Direction = left
                    speed = Range.clip(Math.abs(speed), 0.0, 1.0);
                    frontLeft.setPower(-speed);
                    frontRight.setPower(speed);
                    backLeft.setPower(speed);
                    backRight.setPower(-speed);
                }
                // start motion.

                isMoving = true;

                // keep looping while we are still active, and BOTH motors are running.
                while ((frontLeft.isBusy())) {
                    isMoving = true;
                }

                // Stop all motion;
                frontRight.setPower(0);
                frontLeft.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                isMoving = false;

                // Turn off RUN_TO_POSITION
                frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }


    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range
        robotError = targetAngle - getHeading();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public double getHeading() {
        // TODO check this . . .
        return imu.getIMU().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

}
