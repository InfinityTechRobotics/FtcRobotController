package org.firstinspires.ftc.teamcode.hardware;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
import org.firstinspires.ftc.teamcode.hardware.Constants;

import java.util.function.BooleanSupplier;

import static org.firstinspires.ftc.teamcode.hardware.Constants.WOF_COUNTS_PER_ROTATION;


public class ArmBot {

    // Declare Constants
    public static final double CORE_HEX_EDGES_PER_RADIAN = 288d / (2 * Math.PI);
    public static final double CAROUSEL_POWER = 0.25;

    // TODO check these measurements, i just guessed -_-
    public static final double JOINT_1_LEN = 0.2; //m
    public static final double JOINT_2_LEN = 0.4; //m

    // Declare Actuators
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor carousel = null;
    public DcMotor joint1 = null;
    public Servo joint2 = null;
    private IMU imu = null;

    public boolean isMoving = false;
    public boolean WOFisMoving;
    public boolean isChilling = false;

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
    public void init(HardwareMap hardwareMap, Telemetry tm) {

        // Initialize drive motors
        frontLeft = hardwareMap.get(DcMotor.class, "lf");
        frontRight = hardwareMap.get(DcMotor.class, "rf");
        backLeft = hardwareMap.get(DcMotor.class, "lr");
        backRight = hardwareMap.get(DcMotor.class, "rr");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
        isMoving = false;

        // Initialize carousel motor
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        carousel.setPower(0.0);

        // Initialize arm motors
//        joint1 = hardwareMap.get(DcMotor.class, "joint1");
//
//        joint2 = hardwareMap.get(Servo.class, "joint2");
//
//        joint1.setPower(0d);
//        joint1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        joint2.setPosition(0.0d);

    }

    /**
     * takes end position of end of the arm and calculates the angle of the motors
     * @param xEnd x position of end of arm
     * @param yEnd y position of end of arm
     * @return [target1, target2] returns array of target encoder values
     */
    public int[] armInverseKinematics(double xEnd, double yEnd){

        int joint1Target = 0;
        int joint2Target = 0;
        double alpha1, alpha2, alpha3, theta1, theta2, hyp;

        hyp = Math.sqrt((xEnd*xEnd)+ (yEnd*yEnd));

        alpha1 = Math.atan2(yEnd, xEnd);
        alpha2 = Math.acos(( (JOINT_1_LEN * JOINT_1_LEN) + (hyp * hyp) - (JOINT_2_LEN * JOINT_2_LEN)) / (2 * JOINT_1_LEN * hyp));
        alpha3 = Math.acos(( (JOINT_1_LEN * JOINT_1_LEN) + (JOINT_2_LEN * JOINT_2_LEN) - (hyp * hyp)) / (2 * JOINT_1_LEN * JOINT_2_LEN));

        theta1 = alpha1 + alpha2;
        theta2 = alpha3 - Math.PI;

        joint1Target = (int) (theta1 * CORE_HEX_EDGES_PER_RADIAN);
        joint2Target = (int) ((theta1 + theta2) * CORE_HEX_EDGES_PER_RADIAN);
        return new int[]{joint1Target, joint2Target};

    }

    /**
     * takes angle of joint motors and calculates the position of the end of the arm
     * @param theta1 angle of joint 1 motor
     * @param theta2 angle of joint 2 motor
     * @return [x,y] array representing position of end of arm in x,y plane
     */
    public double[] armForwardKinematics(double theta1, double theta2) {
        double x1, x2, y1, y2;
        x1 = JOINT_1_LEN * Math.cos(theta1);
        y1 = JOINT_1_LEN * Math.sin(theta1);
        x2 = x1 + (JOINT_2_LEN * Math.cos(theta1 + theta2));
        y2 = y1 + (JOINT_2_LEN * Math.sin(theta1 + theta2));
        return new double[]{x2,y2};
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

    public void setWOFPower(double pwr) {
        carousel.setPower(pwr);
    }

    public void rotateWOF (double rotations, double pwr) {

        long targetPos = Math.round(rotations * WOF_COUNTS_PER_ROTATION);
        targetPos = targetPos + carousel.getCurrentPosition();
        carousel.setTargetPosition((int) targetPos);
        carousel.setPower(pwr);

    }

    /**
     *
     * @param initialPwr
     * @param terminalPwr
     * @param duration time in milliseconds
     */
    public void rampWOF(LinearOpMode om, double initialPwr, double terminalPwr, long duration) {
        double period = 100d;
        double diff = terminalPwr - initialPwr;
        double step = diff / 2 * period;
        long stepTime = duration / (long) period;
        double startTime = System.currentTimeMillis();
        double currentPwr = initialPwr;
        do {
            carousel.setPower(currentPwr);
            om.sleep(stepTime);
            currentPwr += step;
        } while(System.currentTimeMillis() < (startTime + duration));
    }

    public boolean isWOFMoving () {
        return carousel.isBusy();
    }

    public void stopWOF() {
        carousel.setPower(0.0);
    }

    public void chill (long milis) {
        try {
            isChilling = true;
            Thread.sleep(milis);
        } catch (Exception e){}
        isChilling = false;
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
     * @param kP  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double kP) {
        return Range.clip(error * kP, -1, 1);
    }

    public double getHeading() {
        // TODO check this . . .
        if(imu == null) return 0d;
        return imu.getIMU().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    public void gyroTurn(Telemetry tm, BooleanSupplier omActive, double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (omActive.getAsBoolean() && !onHeading(speed, angle, Constants.kP_TURN, tm)) {
            // Update telemetry & Allow time for other processes to run.
            tm.update();
        }

    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param kP    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double kP, Telemetry tm) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= 1) { // heading threshold
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, kP);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        frontRight.setPower(rightSpeed);
        frontLeft.setPower(leftSpeed);
        backLeft.setPower(leftSpeed);
        backRight.setPower(rightSpeed);

        // Display it for the driver.
        tm.addData("Target", "%5.2f", angle);
        tm.addData("Err/St", "%5.2f/%5.2f", error, steer);
        tm.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

}
