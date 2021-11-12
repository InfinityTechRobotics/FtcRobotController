package org.firstinspires.ftc.teamcode.hardware;

public class Constants {

    // Encoder counts per revolution at the motor = 28
    // Actual Cartridge Gear Ratios:
    // 3:1 = 2.89:1
    // 4:1 = 3.61:1
    // 5:1 = 5.23:1
    public static final int ENCODER_CPR = 28;
    public static final double GEAR_RATIO_3_TO_1 = 2.89;
    public static final double GEAR_RATIO_4_TO_1 = 3.61;
    public static final double GEAR_RATIO_5_TO_1 = 5.23;
    // Wheel Diameter from AndyMark website
    public static final double WHEEL_DIAMETER = 3.85823;
    // SLIP_FACTOR calculated using actual distance vs. commanded distance
    public static final double SLIP_FACTOR = 12.0 / 11.75;
    public static final double COUNTS_PER_INCH = SLIP_FACTOR*(ENCODER_CPR * GEAR_RATIO_4_TO_1 * GEAR_RATIO_5_TO_1) / (WHEEL_DIAMETER * Math.PI);
    public static final double kP = 0.1d;
    // Width side to side is approximately 14.0 INCHES, wheel to wheel
    // Assume rotating about the centerline, so the circumference of a 360 degree rotation is diameter * pi
    // Use COUNTS_PER_INCH after calculating the circumference
    public static final double ROTATION_SLIP_FACTOR = 1.3;
    public static final double WIDTH = 14.0;
    public static final double COUNTS_PER_DEGREE = ((ROTATION_SLIP_FACTOR*WIDTH*Math.PI)*COUNTS_PER_INCH)/360;
    public static final double STRAFE_SLIP_FACTOR = 1.5;
//    public static final double COUNTS_PER_INCH = ENCODER_CPR / (WHEEL_DIAMETER * Math.PI);
//    public static final double kP = 0.15d;
    public static final double kP_TURN = 0.1d;
    public static final int WOF_COUNTS_PER_ROTATION = 28;
}
