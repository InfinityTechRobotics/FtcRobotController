package org.firstinspires.ftc.teamcode.opmodes.auton;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class TestGyroDrive extends LinearOpMode {

    private Robot robot;

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot();
        robot.init(hardwareMap);
        waitForStart();
        robot.gyroDrive(telemetry, ()-> opModeIsActive(), 0.5, 10, 0);
        robot.gyroTurn(telemetry, () -> opModeIsActive(), 0.25, Math.PI/2);
    }

}
