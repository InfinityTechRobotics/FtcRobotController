
package org.firstinspires.ftc.teamcode.opmodes.auton;

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

@Autonomous(name="Auton RED SOUTH", group="Widebot")

public class AutonRedSouth extends LinearOpMode {

    /* Declare OpMode members. */
    Robot robot = new Robot();
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     FORWARD_SPEED = 0.3;
    static final double     TURN_SPEED    = 0.3;

    String telemetryMessage = "";
    boolean active = false;

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

        // command to strafe to the left 24 inches
        robot.Strafe(FORWARD_SPEED,-24.0);

        while (robot.isMoving) {
            telemetry.addData("Path", "In Progress");
            telemetry.update();
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();

        // Spin the Wheel of Forture

        // Drive forward 18 inches
        robot.Drive(FORWARD_SPEED,18.0);

        while (robot.isMoving) {
            telemetry.addData("Path", "In Progress");
            telemetry.update();
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();

        // Rotate right 90 degrees to face NORTH
        robot.Rotate(TURN_SPEED,90.0);

        while (robot.isMoving) {
            telemetry.addData("Path", "In Progress");
            telemetry.update();
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

}
