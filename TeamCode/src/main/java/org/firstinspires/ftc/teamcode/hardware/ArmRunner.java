package org.firstinspires.ftc.teamcode.hardware;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;

public class ArmRunner extends Thread {

    private final double TUCKED_IN_POS = 0d;

    public enum ArmPosition {
        // TODO tune these
        COLLECT(3, 0.4d),
        BACK_COLLECT(49, 1d),
        LOW(72, 0.75d),
        MID(127, 0.5d),
        HIGH(158, 0.4d);

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
    public static boolean isActive = false;
    private static int numRunning = 0;

    public ArmRunner(Arm arm, ArmPosition target, Telemetry tm) {
        this.arm = arm;
        this.target = target;
        this.tm = tm;
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void run() {
        isActive = true;
        try {

            arm.setJoint2(TUCKED_IN_POS);
            // TODO if pid loop, need to wait for a certain range of values here
            arm.setJoint1(tm, target.getJoint1());
            arm.setJoint2(target.getJoint2());

        } catch(Exception e) {
            e.printStackTrace();
        }
        isActive = false;

    }
}
