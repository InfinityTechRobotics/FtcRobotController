package org.firstinspires.ftc.teamcode.hardware;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;

public class ArmRunner extends Thread {

    private final double TUCKED_IN_POS = 0.75; // TODO this is wrong. pls fix

    public enum ArmPosition {
        // TODO tune these
        COLLECT(0, 0d),
        LOW(0, 0d),
        MID(0, 0d),
        HIGH(0, 0d);

        private int joint1;
        private double joint2;
        ArmPosition(int joint1, double joint2) {
            this.joint1 = joint1;
            this.joint2 = joint2;
        }
        public int getJoint1() { return joint1; }
        public double getJoint2() { return joint2; }

    }

    private Arm arm;
    private ArmPosition target;
    private Telemetry tm;
    private BooleanSupplier omActive;

    public ArmRunner(Arm arm, ArmPosition target, Telemetry tm, BooleanSupplier omActive) {
        this.arm = arm;
        this.target = target;
        this.tm = tm;
        this.omActive = omActive;
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void run() {

        try {

            arm.setJoint2(TUCKED_IN_POS);
            arm.setJoint1(tm, omActive, target.getJoint1());
            arm.setJoint2(target.getJoint2());


        } catch(Exception e) {
            e.printStackTrace();
        }

    }
}
