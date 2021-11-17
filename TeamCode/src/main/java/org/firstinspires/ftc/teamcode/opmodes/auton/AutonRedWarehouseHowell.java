
package org.firstinspires.ftc.teamcode.opmodes.auton;

import android.os.Build;

import androidx.annotation.RequiresApi;

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

@Autonomous(name="Auton RED WAREHOUSE HOWELL", group="Widebot")

public class AutonRedWarehouseHowell extends LinearOpMode {

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


        //Going for Team shipping hub delivery
        robot.Drive(FORWARD_SPEED,14.0);

        while (robot.isMoving) {
            telemetry.addData("Path", "In Progress");
            telemetry.update();
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();

        //Turn Anticlock towards Alliance Shipping Hub
        robot.Rotate(TURN_SPEED,-120.0);

        //Start Linear Slide motor
        robot.moveLiftToPos(telemetry, () -> opModeIsActive(), 0.9d); // TODO tune %

        //Dump the load
        robot.dump();
        sleep(2000);

        robot.carry();

        robot.moveLiftToPos(telemetry, () -> opModeIsActive(), 0.15d);

        robot.unDump();

        //Turn clock towards Alliance Shipping Hub
        robot.Rotate(TURN_SPEED,120.0);

        // Drive Back 24 inches
        robot.Drive(FORWARD_SPEED,-13.0);

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

        // command to strafe to the right 28 inches
        robot.Strafe(FORWARD_SPEED,28.0);

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
