
package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;

/**
    This Op Mode is for use when part of the BLUE ALLIANCE and we take the position to the
    NORTH (closest to the back wall and the Warehouse)

    Path is:
        1. Strafe left X inches until the robot is completely in the BLUE WAREHOUSE
        2. Stop

        This would result in 5 points if executed correctly
 */

@Autonomous(name="Auton BLUE WAREHOUSE", group="Widebot")

public class AutonBlueWarehouse extends LinearOpMode {

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

        //Switch to carry position
        robot.carry();

        //Going for Team shipping hub delivery
        // Drive forward 12 inches
        robot.Drive(FORWARD_SPEED,12.0);
        while (robot.isMoving) {
            telemetry.addData("Path", "In Progress");
            telemetry.update();
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();

        //Turn clockwise(Right)
        robot.Rotate(TURN_SPEED,30.0);

        while (robot.isMoving) {
            telemetry.addData("Path", "In Progress");
            telemetry.update();
        }

        //Start Linear Slide motor
        robot.setSlidePower(0.4);

        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //Dump the load
        robot.dump();



        // 4. Strafe left 24 inches until the robot is completely in the BLUE WAREHOUSE
        //robot.Strafe(FORWARD_SPEED, -24);

        while (robot.isMoving) {
            telemetry.addData("Path", "In Progress");
            telemetry.update();
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();

        // 2. End of Path

    }

}
