
package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;

/**
    This Op Mode is for use when part of the BLUE ALLIANCE and we take the position to the
    SOUTH (closest to the audience and the carousel)

    Path is:
        1. Drive Forward X inches
        2. Rotate -90 degrees
        3. Drive Backward X inches until the robot is touching the front wall
        4. Strafe left X inches until the robot is touching the Blue Carousel
        5. Rotate the WOF X rotations to deliver the duck
        6. Strafe right X inches until the robot is fully inside the BLUE STORAGE UNIT
        7. Stop

        This would result in 10 + 6 + 2 = 18 points if executed correctly
 */

@Autonomous(name="Auton BLUE CAROUSEL", group="Widebot")
@Disabled
public class AutonBlueCarousel extends LinearOpMode {

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

        // 1. Drive forward 18 inches
        robot.Drive(FORWARD_SPEED,18.0);

        while (robot.isMoving) {
            telemetry.addData("Path", "In Progress");
            telemetry.update();
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();

        // 2. Rotate left 90 degrees to face NORTH
        robot.Rotate(TURN_SPEED,-90.0);

        while (robot.isMoving) {
            telemetry.addData("Path", "In Progress");
            telemetry.update();
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();

        // 3. Drive backward 18 inches to touch the front wall
        robot.Drive(FORWARD_SPEED,-18.0);

        while (robot.isMoving) {
            telemetry.addData("Path", "In Progress");
            telemetry.update();
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();

        // 4. Strafe left 12 inches until the robot is touching the BLUE CAROUSEL
        robot.Strafe(FORWARD_SPEED, -12);

        while (robot.isMoving) {
            telemetry.addData("Path", "In Progress");
            telemetry.update();
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();

        // 5. Spin the Wheel of Fortune
        robot.rotateWOF(5.0, 0.3);

        while (robot.isWOFMoving()) {
            telemetry.addData("WOF ", "Rotating");
            telemetry.update();
        }

        robot.stopWOF();
        telemetry.addData("WOF ", "Complete");
        telemetry.update();

        // 6. Strafe right 12 inches until the robot is completely inside the BLUE STORAGE UNIT
        robot.Strafe(FORWARD_SPEED, 12);

        while (robot.isMoving) {
            telemetry.addData("Path", "In Progress");
            telemetry.update();
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();

        // 7. End of Path

    }

}
