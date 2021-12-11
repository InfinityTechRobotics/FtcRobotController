package org.firstinspires.ftc.teamcode.hardware;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;

public class Arm {

    private DcMotor joint1 = null;
    private Servo joint2 = null;
    private Servo claw = null;
    public static final double INTAKE_POWER = 1.0d;

    private DcMotor intake = null;
    private DcMotor carousel = null;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {

        //Initialize joint 1 and 2, claw
        joint1 = hardwareMap.get(DcMotor.class, "joint1");
        joint2 = hardwareMap.get(Servo.class, "joint2");
        claw = hardwareMap.get(Servo.class, "claw");

        //SETTING MOTOR MODE
        joint1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        joint1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        joint1.setPower(0d);
        joint1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //joint2.setPosition(0.0d);

        //close claw
        claw.setPosition(0.8d);

        // Initialize intake motor
        intake = hardwareMap.get(DcMotor.class, "intake");

        // Initialize carousel motor
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        carousel.setPower(0.0);

    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void setJoint1(Telemetry tm, int target) {
        joint1.setTargetPosition(target);
        joint1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (joint1.isBusy()) {
            joint1.setPower(0.5d);
        }
        joint1.setPower(0d);
        joint1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }


    public void setJoint2(double target) {joint2.setPosition(target);}

    public void setClaw(double target) {claw.setPosition(target);}

    public void set(double pwr) {joint1.setPower(0.4d);
    }

    public void setJoin1Power(double pwr) {joint1.setPower(pwr);
    }

    public void setArmJoint1(double pwr) {joint1.setPower(pwr);
    }

    public int getJoint1() {
        return joint1.getCurrentPosition();
    }

    public double getJoint2() {
        return joint2.getPosition();
    }

    public void collect() {intake.setPower(INTAKE_POWER); }

    public void eject() {
        intake.setPower(-INTAKE_POWER);
    }

    public void stopIntake() {
        intake.setPower(0.0);
    }

    public void setWOFPower(double pwr) {
        carousel.setPower(pwr);
    }

}
