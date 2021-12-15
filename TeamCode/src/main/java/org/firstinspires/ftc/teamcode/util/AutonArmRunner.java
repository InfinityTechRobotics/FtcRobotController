package org.firstinspires.ftc.teamcode.util;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PIDFController;
import org.firstinspires.ftc.teamcode.hardware.Arm;

import java.util.function.BooleanSupplier;

public class AutonArmRunner extends Thread {

    /* Singleton instance of the process */
    private static AutonArmRunner INSTANCE = null;

    public static AutonArmRunner getInstance(HardwareMap hardwareMap, Telemetry telemetry, BooleanSupplier omActive) {
        if(INSTANCE == null) INSTANCE = new AutonArmRunner(hardwareMap, telemetry, omActive);
        return INSTANCE;
    }

    /* Arm object */
    private Arm arm;

    /* Misc. tools */
    private BooleanSupplier omActive;
    private double pwr;

    /* PID things */
    private int setpoint = 0;
    private static final PIDFController pid = new PIDFController(0.02d, 0d, 0.00165d, 0d);

    /* Private constructor which starts thread */
    private AutonArmRunner(HardwareMap hardwareMap, Telemetry telemetry, BooleanSupplier omActive) {
        this.arm = new Arm();
        this.omActive = omActive;
        this.arm.init(hardwareMap, telemetry);
        this.start();
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void run() {

        try {

            /* Loops runs once a second forever */
            while(omActive.getAsBoolean()) {

                pwr = pid.calculate(arm.getJoint1(), setpoint);
                arm.setArmJoint1(pwr);

            }

        } catch (Exception e) {
            e.printStackTrace();
        }

    }

    /**
     * Sets the setpoint
     * @param target the desired value of the joint 1 encoder
     */
    public synchronized void setSetpoint(int target) { setpoint = target; }

    public static synchronized void kill() { INSTANCE = null; }

}
