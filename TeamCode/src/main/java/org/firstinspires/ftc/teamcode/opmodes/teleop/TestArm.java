package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.ArmBot;
import org.firstinspires.ftc.teamcode.hardware.ArmRunner;

@TeleOp (name = "TestArm")
public class TestArm extends OpMode {

//    ArmBot armbot = new ArmBot();
    Arm arm = new Arm();

    @Override
    public void init() {

//        armbot.init(hardwareMap, telemetry);
        arm.init(hardwareMap, telemetry);

    }

    @Override
    public void loop() {

        if(gamepad2.x) (new ArmRunner(arm, ArmRunner.ArmPosition.COLLECT, telemetry, () -> true)).start();
        if(gamepad2.a) (new ArmRunner(arm, ArmRunner.ArmPosition.LOW, telemetry, () -> true)).start();
        if(gamepad2.b) (new ArmRunner(arm, ArmRunner.ArmPosition.MID, telemetry, () -> true)).start();
        if(gamepad2.y) (new ArmRunner(arm, ArmRunner.ArmPosition.HIGH, telemetry, () -> true)).start();

        telemetry.addData("Joint 1 Pos: ", () -> arm.getJoint1());
        telemetry.addData("Joint 2 Pos: ", () -> arm.getJoint2());
        telemetry.update();


    }

}
