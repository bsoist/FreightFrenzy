package org.firstinspires.ftc.teamcode;

import android.app.Notification;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class opencv_test extends OpenCvPipeline {
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

    public opencv_test (Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(23, 58, 78);
        Scalar highHSV = new Scalar(32, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat middle = mat.submat(MIDDLE_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        double middleValue = Core.sumElems(middle).val[0] / MIDDLE_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        middle.release();
        right.release();

        telemetry.addData("middle raw value", (int) Core.sumElems(middle).val[0]);
        telemetry.addData("right raw value", (int) Core.sumElems(right).val[0]);

        telemetry.addData("middle percentage", Math.round(middleValue * 100) + "%");
        telemetry.addData("right percentage", Math.round(rightValue * 100) + "%");

        boolean duckMiddle = middleValue > PERCENT_COLOR_THRESHOLD;
        boolean duckRight = rightValue > PERCENT_COLOR_THRESHOLD;

        if (duckMiddle && duckRight) {
            location = Location.NOT_FOUND;
            telemetry.addData("Duck Location", "not found");
        }
        else if (duckMiddle) {
            location = Location.MIDDLE;
            telemetry.addData("Duck Location", "middle");
        }
        else if (duckRight){
            location = Location.RIGHT;
            telemetry.addData("Duck Location", "right");
        }
        else {
            location = Location.LEFT;
            telemetry.addData("Duck Location", "left");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorNone = new Scalar(255, 0, 0);
        Scalar colorDuck = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, MIDDLE_ROI, location == Location.MIDDLE? colorDuck:colorNone);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? colorDuck:colorNone);

//        Imgproc.rectangle(mat, MIDDLE_ROI, colorDuck);
//        Imgproc.rectangle(mat, RIGHT_ROI, colorDuck);

        return mat;

    }

    public Location getLocation() {
        return location;
    }
}
