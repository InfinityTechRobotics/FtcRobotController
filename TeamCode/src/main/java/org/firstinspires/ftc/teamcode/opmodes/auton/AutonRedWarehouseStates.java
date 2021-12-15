package org.firstinspires.ftc.teamcode.opmodes.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.PIDFController;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;

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

    public static final double joint2Lev3DeliverPos = 0.7d;
    public static final double joint2Lev2DeliverPos = 0.7d; // TODO: change this
    public static final double joint2Lev1DeliverPos = 0.7d;
    public static final double joint2TuckPosition = 0.1d;
    public static final double joint2CollectPosition = 0.45d;

    public static final double CLAW_TUCK_POS = 1.0d;
    public static final double CLAW_COLLECT_POS = 0.8d;
    public static final double CLAW_OPEN_POS = 0.8d;

    int armSetpoint = 0;
    private static final PIDFController pid = new PIDFController(0.02d, 0d, 0.00165d, 0d);

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Arm arm = new Arm();
        arm.init(hardwareMap, telemetry);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(15,15))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(25,25))
                .build();


        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineTo(new Vector2d(15,15))
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineTo(new Vector2d(5,5))
                .build();


        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(traj1);

        //set arm setpoint
        double joint1Power = 0.0;

        armSetpoint = 230; // move joint1 to level 3 position
        boolean contWhileLoop = true;

        while (opModeIsActive() && contWhileLoop) {

            joint1Power = pid.calculate(arm.getJoint1(), armSetpoint);
            arm.setArmJoint1(joint1Power);

            if(Math.abs(armSetpoint-arm.getJoint1())<20)
                contWhileLoop=false;

        }

        arm.setJoint2(joint2Lev3DeliverPos);

        drive.followTrajectory(traj2);

        arm.setClaw(CLAW_OPEN_POS);
        sleep(500);

        drive.followTrajectory(traj3);

        arm.setJoint2(0d);
        sleep(500);
        armSetpoint = 150; // move to X position
        contWhileLoop = true;

        while (opModeIsActive() && contWhileLoop) {

            joint1Power = pid.calculate(arm.getJoint1(), armSetpoint);
            arm.setArmJoint1(joint1Power);

            if(Math.abs(armSetpoint-arm.getJoint1())<20)
                contWhileLoop=false;
        }

        drive.followTrajectory(traj4);
        sleep(30000);


        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();
    }

}
