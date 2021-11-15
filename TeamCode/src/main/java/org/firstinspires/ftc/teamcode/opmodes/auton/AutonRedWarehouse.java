
package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;

/**
    This Op Mode is for use when part of the RED ALLIANCE and we take the position to the
    NORTH (closest to the back wall and the Warehouse)

    Path is:
        1. Strafe right X inches until the robot is completely in the RED WAREHOUSE
        2. Stop

        This would result in 5 points if executed correctly
 */

@Autonomous(name="Auton RED WAREHOUSE", group="Widebot")

public class AutonRedWarehouse extends LinearOpMode {

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
        robot.Drive(FORWARD_SPEED,14.0);
        //Turn clockwise(Right)
        robot.Rotate(TURN_SPEED,-120.0);

        //Start Linear Slide motor
       robot.setSlidePower(0.5);

        sleep(5000);

        //Dump the load
        robot.dump();
        sleep(2000);



        // 2. End of Path

    }

}
