
package org.firstinspires.ftc.teamcode.opmodes.auton;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;

/**
    This Op Mode is for use when part of the RED ALLIANCE and we take the position to the
    SOUTH (closest to the audience and the carousel)

    Path is:
        Strafe left x inches until the WOF is touching the carousel
        Rotate the WOF until the duck is delivered (10 points)
        Drive forward x inches
        Either strafe left x inches, or rotate left 90 degrees and drive forward x inches
        this would result in the robot in the RED STORAGE UNIT (3 points if in, 6 points if completely in)
        along with the pre-loaded box (2 points)

        This would result in 10 + 6 + 2 = 18 points if executed correctly
 */

@Autonomous(name="Auton BLUE Carousel Howell", group="Widebot")

public class AutonBlueCarouselHowell extends LinearOpMode {

    /* Declare OpMode members. */
    Robot robot = new Robot();
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     FORWARD_SPEED = 0.3;
    static final double     TURN_SPEED    = 0.3;

    String telemetryMessage = "";
    boolean active = false;

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, telemetry);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();



        telemetry.addData("Status", "WOF is Done");    //
        telemetry.update();



        while (robot.isMoving) {
            telemetry.addData("Path", "In Progress");
            telemetry.update();
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();

        //TEST COde
        //robot.Rotate(TURN_SPEED,-30.0);
        //sleep(30000);

        //TEST ENDS

        // Drive forward 10 inches
        //robot.Drive(FORWARD_SPEED,10.0);
        robot.Drive(FORWARD_SPEED,15.0);

        while (robot.isMoving) {
            telemetry.addData("Path", "In Progress");
            telemetry.update();
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();

        robot.Rotate(TURN_SPEED,-120.0);


        while (robot.isMoving) {
            telemetry.addData("Path", "In Progress");
            telemetry.update();
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();

        //Start Linear Slide motor
        robot.moveLiftToPos(telemetry, () -> opModeIsActive(), 0.9d); // TODO tune %

        //Dump the load
        robot.dump();
        sleep(2000);

        robot.carry();

        robot.moveLiftToPos(telemetry, () -> opModeIsActive(), 0.15d);

        robot.unDump();

        robot.Rotate(TURN_SPEED,50.0);

        //Not Using Ramp WOF for now
        //robot.rampWOF(this, 0.1d, 0.5d, 10000);

        // Drive forward 20 inches
        robot.Drive(FORWARD_SPEED,-25);


        robot.Drive(0.1,-5.0);
        robot.Drive(0.025,-1.0);

        while (robot.isMoving) {
            telemetry.addData("Path", "In Progress");
            telemetry.update();
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();

        //WOF
        robot.setWOFPower(0.7);

        //try back and forth
        robot.Drive(0.025,1.0);
        robot.Drive(0.025,-2.0);
        robot.Drive(0.025,1.0);
        robot.Drive(0.025,-2.0);

        robot.Drive(0.4,6.0);
        // Rotate right 90 degrees to face NORTH
        robot.Rotate(TURN_SPEED,70.0);
        robot.setWOFPower(0.0);

        //sleep(30000);

        robot.Drive(0.6,16.0);
        robot.Rotate(TURN_SPEED,-100.0);
        robot.Drive(0.6,-5.0);


    }

}
