package org.firstinspires.ftc.teamcode.opmodes.teleop;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@TeleOp(name="Test Lift Encoder")
public class TestLiftEncoder extends OpMode {

    private Robot robot = null;

    @Override
    public void init() {
        robot = new Robot();
        robot.init(hardwareMap, telemetry);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void loop() {

        /* Slider */
//        double sliderUp = gamepad2.right_trigger;
//        double sliderDown = gamepad2.left_trigger;
//        robot.setSlidePower(sliderUp - sliderDown);

        if(gamepad2.a) {
            robot.moveLiftToPos(telemetry, ()-> true, .5d);
        }
        if(gamepad2.b) {
            robot.moveLiftToPos(telemetry, ()-> true, 0d);
        }
        if(gamepad2.x) {
            robot.moveLiftToPos(telemetry, ()-> true, 0.9d);
        }

        telemetry.addData("Lift Encoder: ", robot.getLiftEnc());
        telemetry.update();

    }
}
