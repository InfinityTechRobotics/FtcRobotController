
package org.firstinspires.ftc.teamcode.opmodes.auton;

import android.os.Build;

import androidx.annotation.RequiresApi;

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

@Autonomous(name="Auton BLUE WAREHOUSE HOWELL", group="Widebot")

public class AutonBlueWarehouseHowell extends LinearOpMode {

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

        while (robot.isMoving) {
            telemetry.addData("Path", "In Progress");
            telemetry.update();
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();


        // Drive forward 26 inches
        robot.Drive(FORWARD_SPEED,26.0);

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

        // Drive Back 26 inches
        robot.Drive(FORWARD_SPEED,-24.0);

        while (robot.isMoving) {
            telemetry.addData("Path", "In Progress");
            telemetry.update();
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();

        robot.Drive(0.1,-5.0);

        while (robot.isMoving) {
            telemetry.addData("Path", "In Progress");
            telemetry.update();
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();


        // command to strafe to the left 6 inches
        robot.Strafe(FORWARD_SPEED,-26.0);

        while (robot.isMoving) {
            telemetry.addData("Path", "In Progress");
            telemetry.update();
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();

        // Drive forward 20 inches(To Make space for alliance to park in warehouse
        robot.Drive(FORWARD_SPEED,20.0);

        while (robot.isMoving) {
            telemetry.addData("Path", "In Progress");
            telemetry.update();
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();

        // 2. End of Path

    }

}
