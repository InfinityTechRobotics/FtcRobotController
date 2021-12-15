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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PIDFController;
import org.firstinspires.ftc.teamcode.WebcamExample;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name="1-BLUE CAROUSEL STATES", group = "drive")
public class AutonBlueCarouselStates extends LinearOpMode {
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
    boolean contWhileLoop = true;
    String duckPos = "RIGHT";


    int armSetpoint = 0;
    private static final PIDFController pid = new PIDFController(0.02d, 0d, 0.00165d, 0d);
    OpenCvWebcam webcam;
    WebcamExample.SkystoneDeterminationPipeline skystoneDeterminationPipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Arm arm = new Arm();
        arm.init(hardwareMap, telemetry);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "wc"), cameraMonitorViewId);

        skystoneDeterminationPipeline = new WebcamExample.SkystoneDeterminationPipeline();
        webcam.setPipeline(skystoneDeterminationPipeline);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {

                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();


        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Lower Level Shipping Traj
        //--------------------------
        Trajectory traj1l = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(-12,12))
                .build();

        Trajectory traj2l = drive.trajectoryBuilder(traj1l.end())
                .lineTo(new Vector2d(-20,20))
                .build();


        Trajectory traj3l = drive.trajectoryBuilder(traj2l.end())
                .strafeRight(10)
                .build();

        Trajectory traj4l = drive.trajectoryBuilder(traj3l.end())
                .forward(15)
                .build();

        Trajectory traj5l = drive.trajectoryBuilder(traj4l.end())
                .strafeRight(10)
                .build();
        Trajectory traj6l = drive.trajectoryBuilder(traj5l.end())
                .forward(33)
                .build();
        Trajectory traj7l = drive.trajectoryBuilder(traj6l.end())
                .strafeLeft(22)
                .build();
////////////////////////////////////////////////////////////////
        //Middle Level Shipping Traj
        //--------------------------
        Trajectory traj1m = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(-12,12))
                .build();

        Trajectory traj2m = drive.trajectoryBuilder(traj1m.end())
                .lineTo(new Vector2d(-19,19))
                .build();


        Trajectory traj3m = drive.trajectoryBuilder(traj2m.end())
                .strafeRight(12)
                .build();

        Trajectory traj4m = drive.trajectoryBuilder(traj3m.end())
                .back(25)
                .build();

        Trajectory traj5m = drive.trajectoryBuilder(traj4m.end())
                .strafeRight(16)
                .build();
        Trajectory traj6m = drive.trajectoryBuilder(traj5m.end())
                .back(36)
                .build();
        Trajectory traj7m = drive.trajectoryBuilder(traj6m.end())
                .strafeLeft(22)
                .build();
////////////////////////////////////////////////////////////////

        //Top Level Shipping Traj
        //--------------------------
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(-12,12))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(-20,20))
                .build();


        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeRight(12)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .back(25)
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .strafeRight(16)
                .build();
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .back(36)
                .build();
        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .strafeLeft(22)
                .build();
////////////////////////////////////////////////////

        waitForStart();

        ElapsedTime runtime = new ElapsedTime();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            duckPos = skystoneDeterminationPipeline.getAnalysis().toString();
            telemetry.addData("Duck Position XXX", duckPos);
            telemetry.update();
        }

        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            duckPos = skystoneDeterminationPipeline.getAnalysis().toString();
            telemetry.addData("Duck Position YYY", duckPos);
            telemetry.update();
        }

        telemetry.addData("Final Duck Position", duckPos);
        telemetry.update();

        telemetry.addData("Final Duck Position", skystoneDeterminationPipeline.getAnalysis());
        telemetry.update();

        if (isStopRequested()) return;

        webcam.stopStreaming();

        //set arm setpoint
        double joint1Power = 0.0;

        //Decide path based on shipping element position
        //switch(skystoneDeterminationPipeline.getAnalysis()){
        if(duckPos=="LEFT"){
            telemetry.addData("Duck Position 111", duckPos);
            telemetry.update();

            drive.followTrajectory(traj1l);

            armSetpoint = 120; // move joint1 to lower level
            contWhileLoop = true;


            while (opModeIsActive() && contWhileLoop) {

                joint1Power = pid.calculate(arm.getJoint1(), armSetpoint);
                arm.setArmJoint1(joint1Power);

                if (Math.abs(armSetpoint - arm.getJoint1()) < 20)
                    contWhileLoop = false;

            }

            sleep(10000);
            arm.setJoint2(0.7d);


            drive.followTrajectory(traj2l);

            arm.setClaw(CLAW_OPEN_POS);
            sleep(500);

            drive.followTrajectory(traj3l);

            arm.setJoint2(joint2TuckPosition);

            sleep(1000);
            ;
            armSetpoint = 0; // Come to Base
            contWhileLoop = true;

            runtime = new ElapsedTime();

            while (opModeIsActive() && (runtime.seconds() < 2)) {

                joint1Power = pid.calculate(arm.getJoint1(), armSetpoint);
                arm.setArmJoint1(joint1Power);

                if (Math.abs(armSetpoint - arm.getJoint1()) < 20)
                    contWhileLoop = false;
            }


            sleep(20000);
            drive.followTrajectory(traj4l);
            drive.followTrajectory(traj5l);
            drive.followTrajectory(traj6l);
            drive.followTrajectory(traj7l);

        }
        if(duckPos=="CENTER") {
            telemetry.addData("Duck Position 2222", duckPos);
            telemetry.update();


            drive.followTrajectory(traj1m);

            armSetpoint = 165; // move joint1 to middle level
            contWhileLoop = true;


            while (opModeIsActive() && contWhileLoop) {

                joint1Power = pid.calculate(arm.getJoint1(), armSetpoint);
                arm.setArmJoint1(joint1Power);

                if (Math.abs(armSetpoint - arm.getJoint1()) < 20)
                    contWhileLoop = false;

            }

            arm.setJoint2(0.8d);

            drive.followTrajectory(traj2m);

            arm.setClaw(CLAW_OPEN_POS);
            sleep(500);

            drive.followTrajectory(traj3m);

            arm.setJoint2(joint2TuckPosition);

            sleep(1000);
            armSetpoint = 0; // Come to Base
            contWhileLoop = true;

            runtime = new ElapsedTime();

            //while (opModeIsActive() && contWhileLoop) {
            while (opModeIsActive() && (runtime.seconds() < 2)) {

                joint1Power = pid.calculate(arm.getJoint1(), armSetpoint);
                arm.setArmJoint1(joint1Power);

                if (Math.abs(armSetpoint - arm.getJoint1()) < 15)
                    contWhileLoop = false;
            }


            drive.followTrajectory(traj4m);
            drive.followTrajectory(traj5m);
            drive.followTrajectory(traj6m);
            drive.followTrajectory(traj7m);
            sleep(20000);
        }
        if(duckPos=="RIGHT") {

            telemetry.addData("Duck Position 333", duckPos);
            telemetry.update();
            drive.followTrajectory(traj1);

            armSetpoint = 230; // move joint1 to level 3 position
            boolean contWhileLoop = true;

            //runtime = new ElapsedTime();

            while (opModeIsActive() && contWhileLoop) {
                //while (opModeIsActive() && (runtime.seconds() < 2.0)) {

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

            arm.setJoint2(joint2TuckPosition);
            sleep(1000);

            armSetpoint = 0; // Come to Base
            contWhileLoop = true;

            runtime = new ElapsedTime();

            //while (opModeIsActive() && contWhileLoop) {
            while (opModeIsActive() && (runtime.seconds() < 2.0)) {

                joint1Power = pid.calculate(arm.getJoint1(), armSetpoint);
                arm.setArmJoint1(joint1Power);

                if(Math.abs(armSetpoint-arm.getJoint1())<20)
                    contWhileLoop=false;
            }



            drive.followTrajectory(traj4);
            drive.followTrajectory(traj5);
            drive.followTrajectory(traj6);
            drive.followTrajectory(traj7);

            sleep(20000);

        }


        webcam.stopStreaming();

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();
    }

}
