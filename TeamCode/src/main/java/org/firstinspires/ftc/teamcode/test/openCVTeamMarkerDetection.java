package org.firstinspires.ftc.teamcode.test;

import android.app.Notification;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class openCVTeamMarkerDetection extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();

    public enum Location {
        RIGHT,
        MIDDLE,
        LEFT,
        NOT_FOUND
    }
    private Location location;

    static final Rect MIDDLE_ROI = new Rect(
            new Point(10, 60),
            new Point(160, 200));
    static final Rect RIGHT_ROI = new Rect(
            new Point(170, 60),
            new Point(315, 200));

    static double PERCENT_COLOR_THRESHOLD = .15;

    public openCVTeamMarkerDetection (Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        //14296 green
        //H > 155 - 165
        //S > 40 - 100%
        //V > 17 - 45%
//        double lowHue = 154;//out of 360
//        double lowSat = .2; //percent
//        double lowVal = .1; //percent
//        double highHue = 157;//out of 360
//        double highSat = .6;  //percent
//        double highVal = .3; //percent

        //14296 white
//        double lowHue = 0;
//        double lowSat = 0;
//        double lowVal = .1;
//        double highHue = 0;
//        double highSat = 1;
//        double highVal = 1;

        //ftc yellow
//        double lowHue = 23*2;
//        double lowSat = 58/255;
//        double lowVal = 78/255;
//        double highHue = 32*2;
//        double highSat = 255*255;
//        double highVal = 255*255;

        //14296 blue target > 226, 83%, 41%
        double blue_lowHue = 221;
        double blue_lowSat = .6;
        double blue_lowVal = .15;
        double blue_highHue = 231;
        double blue_highSat = 1;
        double blue_highVal = .7;

        //14296 red target > 0, 98%, 67%
        double red_lowHue = 0;
        double red_lowSat = .6;
        double red_lowVal = 0;
        double red_highHue = 6;
        double red_highSat = 1;
        double red_highVal = 1;

        double hueConv = .5; //(out of 180)
        double satConv = 255; //(out of 255)
        double valConv = 255; //(out of 255)

        blue_lowHue *= hueConv;
        blue_highHue *= hueConv;
        red_lowHue *= hueConv;
        red_highHue *= hueConv;

        blue_lowSat *= satConv;
        blue_highSat *= satConv;
        red_lowSat *= satConv;
        red_highSat *= satConv;

        blue_lowVal *= valConv;
        blue_highVal *= valConv;
        red_lowVal *= valConv;
        red_highVal *= valConv;

        Scalar blue_lowHSV = new Scalar(blue_lowHue, blue_lowSat, blue_lowVal);
        Scalar blue_highHSV = new Scalar(blue_highHue, blue_highSat, blue_highVal);

        Scalar red_lowHSV = new Scalar(red_lowHue, red_lowSat, red_lowVal);
        Scalar red_highHSV = new Scalar(red_highHue, red_highSat, red_highVal);

//        Core.inRange(mat, blue_lowHSV, blue_highHSV, mat);
        Core.inRange(mat, red_lowHSV, red_highHSV, mat);

        Mat middle = mat.submat(MIDDLE_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        double middleValue = Core.sumElems(middle).val[0] / MIDDLE_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        middle.release();
        right.release();

//        telemetry.addData("middle raw value", (int) Core.sumElems(middle).val[0]);
//        telemetry.addData("right raw value", (int) Core.sumElems(right).val[0]);

//        telemetry.addData("middle percentage", Math.round(middleValue * 100) + "%");
//        telemetry.addData("right percentage", Math.round(rightValue * 100) + "%");

        boolean duckMiddle = middleValue > PERCENT_COLOR_THRESHOLD;
        boolean duckRight = rightValue > PERCENT_COLOR_THRESHOLD;

        if (duckMiddle && duckRight) {
            location = Location.NOT_FOUND;
//            telemetry.addData("Duck Location", "not found");
        }
        else if (duckMiddle) {
            location = Location.MIDDLE;
//            telemetry.addData("Duck Location", "middle");
        }
        else if (duckRight){
            location = Location.RIGHT;
//            telemetry.addData("Duck Location", "right");
        }
        else {
            location = Location.LEFT;
//            telemetry.addData("Duck Location", "left");
        }
//        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorNone = new Scalar(255, 0, 0);
        Scalar colorDuck = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, MIDDLE_ROI, location == Location.MIDDLE? colorDuck:colorNone);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? colorDuck:colorNone);

        return mat;
    }

    public Location getLocation() {
        return location;
    }
}
