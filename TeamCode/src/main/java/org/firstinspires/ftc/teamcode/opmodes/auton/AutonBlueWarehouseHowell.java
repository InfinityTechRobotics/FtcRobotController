
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

@Autonomous(name="2-BLUE WAREHOUSE HOWELL", group="Widebot")

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

        // Drive forward 15 inches
        robot.Drive(FORWARD_SPEED,12.0);

        while (robot.isMoving) {
            telemetry.addData("Path", "In Progress");
            telemetry.update();
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();

        //Turn Anticlock away from Alliance Shipping Hub
        robot.Rotate(TURN_SPEED,-25.0);

        while (robot.isMoving) {
            telemetry.addData("Path", "In Progress");
            telemetry.update();
        }

        sleep(1000);

        telemetry.addData("Path", "Complete");
        telemetry.update();

        //Start Linear Slide motor
        robot.moveLiftToPos(telemetry, () -> opModeIsActive(), 0.9d); // TODO tune %

        //Dump the load(Test Middle Dump Position
        robot.dump();
        sleep(2000);

        robot.carry();

        robot.moveLiftToPos(telemetry, () -> opModeIsActive(), 0.15d);

        robot.unDump();

        sleep(1000);

        robot.Rotate(TURN_SPEED, 30.0);

        // Drive Back 26 inches
        robot.Drive(FORWARD_SPEED,-10);
        robot.Drive( 0.1, -7);
        //robot.Rotate(0.1, 10.0);

        while (robot.isMoving) {
            telemetry.addData("Path", "In Progress");
            telemetry.update();
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();

        robot.Strafe(0.2,-15);
        robot.Drive(0.2 , -8);
        //robot.Rotate(0.1, 5.0);
        //robot.Drive(0.1 , -3);
        robot.Strafe(0.2, -14);
        robot.Drive(0.2 , -8);
        robot.Strafe(0.12, -14);
        robot.Drive(FORWARD_SPEED, 12);

        while (robot.isMoving) {
            telemetry.addData("Path", "In Progress");
            telemetry.update();
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();

        // 2. End of Path

    }

}
