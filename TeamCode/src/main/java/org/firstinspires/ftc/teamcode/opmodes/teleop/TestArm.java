package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PIDFController;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.ArmBot;
import org.firstinspires.ftc.teamcode.hardware.ArmRunner;

@TeleOp (name = "TestArm-14")
@Disabled
public class TestArm extends OpMode {

    Arm arm = new Arm();

    int armSetpoint = 0;

    private static final PIDFController pid = new PIDFController(0.04d, 0d, 0.004d, 0d);

    @Override
    public void init() {

        arm.init(hardwareMap, telemetry);

        //arm.setJoint2(0d);

    }

    @Override
    public void loop() {

        // set arm setpoint
        double joint1Power = 0.0;



        if(gamepad2.x) {
            //RANDOM TEST POSITION, REPLACE LATER
            armSetpoint = 100;
        }
        if(gamepad2.a) {
            //LEVEL 3
            armSetpoint = 220;
        }
        if(gamepad2.b) {
            // COLLECT POSITION
            armSetpoint = 0;
        }
        if(gamepad2.y){
            //CLAW CLOSE
            arm.setClaw(1d);
        }

        if(gamepad2.dpad_up) {
            //CLAW OPEN
            arm.setClaw(0.6d);
        }

        joint1Power = pid.calculate(arm.getJoint1(), armSetpoint);
        arm.setArmJoint1(joint1Power);
        arm.setJoint2(0.0);
        telemetry.addData("Joint Power", joint1Power);
        telemetry.addData("getJoint1", arm.getJoint1());
        telemetry.addData("armSetPoint", armSetpoint);
        telemetry.addData("getJoint1AfterFunctionCall", arm.getJoint1());
        telemetry.update();


        /*
        if(gamepad2.x) (new ArmRunner(arm, ArmRunner.ArmPosition.COLLECT, telemetry)).start();
        if(gamepad2.a) (new ArmRunner(arm, ArmRunner.ArmPosition.LOW, telemetry)).start();
        if(gamepad2.b) (new ArmRunner(arm, ArmRunner.ArmPosition.MID, telemetry)).start();
        if(gamepad2.y) (new ArmRunner(arm, ArmRunner.ArmPosition.HIGH, telemetry)).start();
        if(gamepad2.start) (new ArmRunner(arm, ArmRunner.ArmPosition.BACK_COLLECT, telemetry)).start();
*/

        /* Intake */
        boolean rb = gamepad2.right_bumper;
        boolean lb = gamepad2.left_bumper;
        // if right bumper is pressed, rb value = true
        if(rb) arm.collect();
        // if left bumper is pressed, lb value = true
        if(lb) arm.eject();
        // if both rb & lb are false, neither bumper is pressed, so stop intake
        if(!(rb && lb)) arm.stopIntake();

        telemetry.clearAll();
        telemetry.addData("Joint 1 Pos: ", () -> arm.getJoint1());
        telemetry.addData("Joint 2 Pos: ", () -> arm.getJoint2());
        telemetry.update();

    }

}
