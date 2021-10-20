package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="My OpMode")
public class ExampleOpMode extends OpMode {

    // Declare Hardware
    Robot myRobot;

    // Constructor
    public ExampleOpMode() {

        // Instantiate Objects
        myRobot = new Robot();

    }

    @Override
    public void init() {

        // Initialize Hardware
        myRobot.init(hardwareMap);

    }

    @Override
    public void loop() {

        // Take Input From Driver or Sensor

        // Sent Input to Actuator

    }

}
