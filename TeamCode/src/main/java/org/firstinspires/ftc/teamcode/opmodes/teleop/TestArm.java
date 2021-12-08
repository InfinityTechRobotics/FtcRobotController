package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PIDFController;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.ArmBot;
import org.firstinspires.ftc.teamcode.hardware.ArmRunner;

@TeleOp (name = "TestArm")
public class TestArm extends OpMode {

    Arm arm = new Arm();

    int armSetpoint = 0;

    private static final PIDFController pid = new PIDFController(0d, 0d, 0d, 0d);

    @Override
    public void init() {

        arm.init(hardwareMap, telemetry);

        arm.setJoint2(0d);

    }

    @Override
    public void loop() {

        // set arm setpoint
        armSetpoint = 0; // TODO take this value from a button or something
//        arm.set(pid.calculate(arm.getJoint1(), armSetpoint)); // TODO implement this

        if(gamepad2.x) (new ArmRunner(arm, ArmRunner.ArmPosition.COLLECT, telemetry)).start();
        if(gamepad2.a) (new ArmRunner(arm, ArmRunner.ArmPosition.LOW, telemetry)).start();
        if(gamepad2.b) (new ArmRunner(arm, ArmRunner.ArmPosition.MID, telemetry)).start();
        if(gamepad2.y) (new ArmRunner(arm, ArmRunner.ArmPosition.HIGH, telemetry)).start();
        if(gamepad2.start) (new ArmRunner(arm, ArmRunner.ArmPosition.BACK_COLLECT, telemetry)).start();

        telemetry.clearAll();
        telemetry.addData("Joint 1 Pos: ", () -> arm.getJoint1());
        telemetry.addData("Joint 2 Pos: ", () -> arm.getJoint2());
        telemetry.update();

    }

}
