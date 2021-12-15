package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.PIDFController;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(name = "1-State Practice OpMode")
public class MainTeleOpModeState extends LinearOpMode {

    int armSetpoint = 0;

    private static final PIDFController pid = new PIDFController(0.02d, 0d, 0.00165d, 0d);
    public static final double joint2Lev3DeliverPos = 0.7d;
    public static final double joint2Lev2DeliverPos = 0.7d; // TODO: change this
    public static final double joint2Lev1DeliverPos = 0.7d;
    public static final double joint2TuckPosition = 0.1d;
    public static final double joint2CollectPosition = 0.45d;

    public static final double CLAW_TUCK_POS = 1.0d;
    public static final double CLAW_COLLECT_POS = 0.8d;
    public static final double CLAW_OPEN_POS = 0.8d;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Arm arm = new Arm();

        arm.init(hardwareMap, telemetry);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.right_stick_y,
                            -gamepad1.right_stick_x,
                            -gamepad1.left_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            /* Added for arm control on gamepad 2 */

            // set arm setpoint
            double joint1Power = 0.0;

            if(gamepad2.x) {
                //Arm - Transfer Position before going to base
                arm.setJoint2(0d);
                armSetpoint = 150;
            }
            if(gamepad2.y) {
                //Arm - Level 1 Position, Shared Shipping Hub
                armSetpoint = 150;
                arm.setJoint2(joint2Lev1DeliverPos);
            }
            if(gamepad2.a) {
                //Arm - LEVEL 3 Position
                armSetpoint = 235; // move joint1 to level 3 position
                arm.setJoint2(joint2Lev3DeliverPos);

            }
            if(gamepad2.b) {
                // Arm goes to Tuck Position inside Robot
                armSetpoint = 0;
            }
            //Game Pad1 - After Deliver
            if(gamepad1.b){
                //CLAW CLOSE
                arm.setClaw(CLAW_TUCK_POS);
            }
            //Game Pad1 - Deliver
            if(gamepad1.a) {
                //CLAW OPEN and CLAW COLLECT is same
                arm.setClaw(CLAW_OPEN_POS);
            }
            //Arm Collect Position inside Robot - Open Claw and bring join2 down
            if(gamepad2.dpad_down) {
                arm.setClaw(CLAW_COLLECT_POS);
                arm.setJoint2(joint2CollectPosition);
                //sleep(500);
                //arm.setJoin1Power(-0.8d);
            }
            //Arm Tuck Position inside Robot - close claw and bring joint2 to tuck position
            if(gamepad2.dpad_up) {
                arm.setClaw(CLAW_TUCK_POS);
                sleep(500);
                arm.setJoint2(joint2TuckPosition);
            }

            /* Wheel of Fortune */
            double x = -gamepad2.left_stick_x;
            arm.setWOFPower(x);

            /* Intake */
            boolean rb = gamepad2.right_bumper;
            boolean lb = gamepad2.left_bumper;
            // if right bumper is pressed, rb value = true
            if(rb) arm.collect();
            // if left bumper is pressed, lb value = true
            if(lb) arm.eject();
            // if both rb & lb are false, neither bumper is pressed, so stop intake
            if(!rb && !lb) arm.stopIntake();

            joint1Power = pid.calculate(arm.getJoint1(), armSetpoint);
            arm.setArmJoint1(joint1Power);

            telemetry.addData("Joint Power", joint1Power);
            telemetry.addData("getJoint1", arm.getJoint1());
            telemetry.addData("armSetPoint", armSetpoint);
            telemetry.addData("getJoint1AfterFunctionCall", arm.getJoint1());
            telemetry.addData("Joint 2 position",arm.getJoint2());
            telemetry.update();

        }
    }
}
