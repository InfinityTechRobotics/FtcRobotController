package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.PIDFController;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.util.AutonArmRunner;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name="TEST ARM THREAD", group = "TESTS")
@Disabled
public class TestArmThread extends LinearOpMode {

    public static double FORWARD_DISTANCE = 28; // in - Towards Warehouse
    public static double STRAFE_LEFT_DISTANCE = 20; // in - Towards Team Shipping Hub
    public static double STRAFE_RIGHT_DISTANCE = 19.5; // in - Towards Team Shipping Hub
    public static double IN_WAREHOUSE_DISTANCE = 16; // in - Towards Team Shipping Hub

    public static final double joint2Lev3DeliverPos = 0.7d;
    public static final double joint2Lev2DeliverPos = 0.7d; // TODO: change this
    public static final double joint2Lev1DeliverPos = 0.7d;
    public static final double joint2TuckPosition = 0.1d;
    public static final double joint2CollectPosition = 0.45d;

    public static final double CLAW_TUCK_POS = 1.0d;
    public static final double CLAW_COLLECT_POS = 0.8d;
    public static final double CLAW_OPEN_POS = 0.8d;

    private int armSetpoint = 0;
    private AutonArmRunner arm = null;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

        arm = AutonArmRunner.getInstance(hardwareMap, telemetry, this::opModeIsActive);


        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(traj1);



        armSetpoint = 235; // move joint1 to level 3 position
        arm.setSetpoint(armSetpoint);





        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);


        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        AutonArmRunner.kill();

    }

}
