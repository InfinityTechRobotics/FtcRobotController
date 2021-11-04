package org.firstinspires.ftc.teamcode.hardware;

public class Constants {

    public static final int ENCODER_CPR = 1120;
    public static final double WHEEL_DIAMETER = 3.85823;
    public static final double COUNTS_PER_INCH = ENCODER_CPR / (WHEEL_DIAMETER * Math.PI);
    public static final double kP = 0.15d;
    public static final double kP_TURN = 0.1d;

}
