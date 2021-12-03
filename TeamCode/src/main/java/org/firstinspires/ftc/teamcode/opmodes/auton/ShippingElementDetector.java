package org.firstinspires.ftc.teamcode.opmodes.auton;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ShippingElementDetector extends OpenCvPipeline {

    //TODO check this...
    final double PERCENT_COLOR_THRESHOLD = 0.2;

    Telemetry telemetry;

    private Mat barcode_Finder = new Mat();

    // TODO add points later
    final static Rect LEFT_BARCODE_ROI = new Rect(new Point(), new Point());
    final static Rect RIGHT_BARCODE_ROI = new Rect(new Point(), new Point());

    public ShippingElementDetector(Telemetry t) { telemetry = t; }

    String shipping_Element_Location = "left";

    @Override
    public final Mat processFrame(Mat input) {

        Imgproc.cvtColor(barcode_Finder, barcode_Finder, Imgproc.COLOR_RGB2HSV);

        Scalar lowRedHSV = new Scalar(161, 155, 84);
        Scalar highRedHSV = new Scalar(171, 255, 255);

        Core.inRange(barcode_Finder, lowRedHSV, highRedHSV, barcode_Finder);

        Mat left_Barcode_Rectangle = barcode_Finder.submat(LEFT_BARCODE_ROI);
        Mat right_Barcode_Rectangle = barcode_Finder.submat(RIGHT_BARCODE_ROI);



        double leftValue = Core.sumElems(left_Barcode_Rectangle).val[0] / LEFT_BARCODE_ROI.area() / 255;
        double rightValue = Core.sumElems(right_Barcode_Rectangle).val[0] / RIGHT_BARCODE_ROI.area() / 255;

        left_Barcode_Rectangle.release();
        right_Barcode_Rectangle.release();

        telemetry.addData("Left raw value", Core.sumElems(left_Barcode_Rectangle).val[0] );
        telemetry.addData("Right raw value", Core.sumElems(right_Barcode_Rectangle).val[0] );
        telemetry.addData("Left percentage", Math.round(leftValue*100) + "%" );
        telemetry.addData("Right percentage", Math.round(rightValue*100) + "%" );

        boolean In_Left_Barcode = leftValue < PERCENT_COLOR_THRESHOLD;
        boolean In_Middle_Barcode = rightValue < PERCENT_COLOR_THRESHOLD;

        if( In_Left_Barcode) {

            // In left barcode
            shipping_Element_Location = "left";
            telemetry.addData("Shipping element location", shipping_Element_Location);

        } else if (In_Middle_Barcode) {

            // In middle barcode
            shipping_Element_Location = "middle";
            telemetry.addData("Shipping element location", shipping_Element_Location);

        } else {

            // in right barcode
            shipping_Element_Location = "right";
            telemetry.addData("Shipping element location", shipping_Element_Location);

        }

        telemetry.update();

        Imgproc.cvtColor(barcode_Finder, barcode_Finder, Imgproc.COLOR_GRAY2RGB);

        return barcode_Finder;
    }
}
