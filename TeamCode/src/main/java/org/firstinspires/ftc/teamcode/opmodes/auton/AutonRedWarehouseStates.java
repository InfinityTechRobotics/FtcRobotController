package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name="4-RED WAREHOUSE STATES", group = "drive")
public class AutonRedWarehouseStates extends LinearOpMode {
    public static double FORWARD_DISTANCE = 28; // in - Towards Warehouse
    public static double STRAFE_LEFT_DISTANCE = 20; // in - Towards Team Shipping Hub
    public static double STRAFE_RIGHT_DISTANCE = 19.5; // in - Towards Team Shipping Hub
    public static double IN_WAREHOUSE_DISTANCE = 16; // in - Towards Team Shipping Hub

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(STRAFE_LEFT_DISTANCE)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeRight(STRAFE_RIGHT_DISTANCE)
                .build();


        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .forward(FORWARD_DISTANCE)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .strafeLeft(IN_WAREHOUSE_DISTANCE)
                .build();


        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
