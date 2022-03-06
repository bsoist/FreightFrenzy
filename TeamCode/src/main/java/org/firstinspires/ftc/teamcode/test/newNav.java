/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Locale;

@Autonomous(name = "NewNav", group = "", preselectTeleOp = "NewIntake")
//@Disabled
public class newNav extends LinearOpMode {
    OpenCvCamera Camera;
    openCVTeamMarkerDetection pipeline;
    openCVTeamMarkerDetection.Location snapshotAnalysis = openCVTeamMarkerDetection.Location.RIGHT; // default

    private DcMotor bLeft = null;
    private DcMotor bRight = null;
    private DcMotor fLeft = null;
    private DcMotor fRight = null;
    private DcMotor elbow = null;
    private DcMotor shoulder = null;
    private Servo   claw     = null;
    private CRServo rightIntake = null;
    private CRServo leftIntake  = null;
    private DcMotor ttMotor = null;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    //global variables
    static final double     DRIVE_SPEED             = 0.4;     // (40% of 2500 ticks/rev = 1000)
    static final double     ARC_SPEED               = 0.4;
    static final double     TURN_SPEED              = 0.4;

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = .05;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = .1;     // Larger is more responsive, but also less stable

    static final double robotRadius = 40.5;
    static final double radiusHubtoRightSide = 34.5;

    //arm preset levels
    boolean bottomLevel = false;
    boolean middleLevel = false;
    boolean topLevel = false;

    //arm variables
    int shoulderTopMid = -555;
    int shoulderBot = 0; //NEED A VALUE
    int shoulderLow = -838;
    int shoulderPickup = -1065;
    int elbowTop = 250;
    int elbowMid = 130;
    int elbowBot = 41;
    int elbowLow = 75;
    int elbowPickup = 128;

    @Override
    public void runOpMode()  throws InterruptedException {
        //camera initialization
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName Webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(Webcam, cameraMonitorViewId);

        openCVTeamMarkerDetection detector = new openCVTeamMarkerDetection(telemetry);

        camera.setPipeline(detector);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        //motor initialization
        DcMotorEx bLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx bRight = hardwareMap.get(DcMotorEx.class, "backRight");
        DcMotorEx fLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx fRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx elbow = hardwareMap.get(DcMotorEx.class, "elbowArmMotor");
        DcMotorEx shoulder = hardwareMap.get(DcMotorEx.class, "shoulderArmMotor");
        claw = hardwareMap.get(Servo.class, "claw");
        rightIntake = hardwareMap.get(CRServo.class, "RightServo");
        leftIntake = hardwareMap.get(CRServo.class, "LeftServo");
        ttMotor = hardwareMap.get(DcMotorEx.class, "turnTableMotor");

        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        bRight.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.REVERSE);
        leftIntake.setDirection(CRServo.Direction.REVERSE);

        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //imu initialization
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //variables

        String[][] barcode = new String[][]{{"left", "bottom level"}, {"middle", "mid level"}, {"right", "top level"}};
        String drive = barcode[2][0]; //may not need
        String level = barcode[2][1];

        boolean intake = false;
        boolean output = false;

        boolean redWarehouse = false;
        boolean redCarousel = false;
        boolean blueWarehouse = false;
        boolean blueCarousel = false;


        //telemetry calling for gyro
        composeTelemetry();

        //pre match setup during initialization period
        while (!isStarted() && !isStopRequested()) {
            if (gamepad2.left_trigger > gamepad2.right_trigger) {
                intake = false;
                output = true;
            } else if (gamepad2.right_trigger > gamepad2.left_trigger) {
                intake = true;
                output = false;
            } else if (gamepad2.right_trigger > -.15 && gamepad2.left_trigger > -.15) {
                intake = false;
                output = false;
                rightIntake.setPower(0);
                leftIntake.setPower(0);
            }
            double intakePower = gamepad2.right_trigger;
            double outputPower = gamepad2.left_trigger;
            if (intake) {
                rightIntake.setPower(intakePower);
                leftIntake.setPower(intakePower);
            }
            if (output) {
                rightIntake.setPower(-outputPower);
                leftIntake.setPower(-outputPower);
            }

            double armSpeed = .75;
            double elbowPower = Range.clip(gamepad2.left_stick_y, -1.0, 1.0) * armSpeed;
            double shoulderPower = Range.clip(gamepad2.right_stick_y, -1.0, 1.0) * armSpeed;
            shoulder.setPower(shoulderPower);
            elbow.setPower(elbowPower);

            if (gamepad2.right_bumper) {
                elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }//reset arm to zero
            int elbowCurrentPosition = elbow.getCurrentPosition();
            int shoulderCurrentPosition = shoulder.getCurrentPosition();
            telemetry.addData("Shoulder Position", "%d", shoulderCurrentPosition);
            telemetry.addData("ELbow Position", "%d", elbowCurrentPosition);


            if (gamepad2.left_bumper) {
                imu.initialize(parameters);
            }//reset heading

            if (gamepad2.y) {
                redWarehouse = true;
                redCarousel = false;
                blueWarehouse = false;
                blueCarousel = false;
            } else if (gamepad2.b) {
                redWarehouse = false;
                redCarousel = true;
                blueWarehouse = false;
                blueCarousel = false;
            }
            if (gamepad2.x) {
                redWarehouse = false;
                redCarousel = false;
                blueWarehouse = true;
                blueCarousel = false;
            }
            if (gamepad2.a) {
                redWarehouse = false;
                redCarousel = false;
                blueWarehouse = false;
                blueCarousel = true;
            }
            telemetry.addData("Analysis", pipeline.getLocation());
            telemetry.update();
//            sleep(50);
        }

        /*
         * The START command just came in: snapshot the current analysis now
         * for later use. We must do this because the analysis will continue
         * to change as the camera view changes once the robot starts moving!
         */

        snapshotAnalysis = pipeline.getLocation();

        telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
        telemetry.update();

        switch (snapshotAnalysis)
        {
            case LEFT:
                topLevel = false;
                middleLevel = false;
                bottomLevel = true;

                break;
            case MIDDLE:
                topLevel = false;
                middleLevel = true;
                bottomLevel = false;

//                runStraight(30, DRIVE_SPEED);
//                gyroTurn(-90, TURN_SPEED);
//                gyroHold(-90, TURN_SPEED, .25);
//                runStraight(30, DRIVE_SPEED);
//                gyroTurn(-180, TURN_SPEED);
//                gyroHold(-180, TURN_SPEED, .25);
//                runStraight(30, DRIVE_SPEED);
//                gyroTurn(90, TURN_SPEED);
//                gyroHold(90, TURN_SPEED, .25);
//                runStraight(30, DRIVE_SPEED);
//                gyroTurn(0, TURN_SPEED);
//                gyroHold(0, TURN_SPEED, .25);

                break;
            case RIGHT:
                topLevel = true;
                middleLevel = false;
                bottomLevel = false;

//                newnew_arcLeft(90, 35); // drive into warehouse
//                gyroHold(-90, ARC_SPEED, .5);

                break;
            case NOT_FOUND:
                topLevel = false;
                middleLevel = false;
                bottomLevel = false;
                telemetry.addData("Error", "OpenCV reads not found?");
                break;
        } //detecting marker on barcode
        camera.stopStreaming();

        if      (redWarehouse)  { run_redWarehouse() ; }
        else if (redCarousel)   { run_redCarousel()  ; }
        else if (blueWarehouse) { run_blueWarehouse(); }
        else if (blueCarousel)  { run_blueCarousel() ; }
    }

    public void runMotorsPower(double leftPower, double rightPower, double speed, long durationMS) {
        bLeft.setPower(leftPower * speed);
        bRight.setPower(rightPower * speed);
        fLeft.setPower(leftPower * speed);
        fRight.setPower(rightPower * speed);
        sleep(durationMS);
    }

    public void runMotorsVel(double leftVelocity, double rightVelocity, double scale/*long durationMS*/) {
        DcMotorEx bLeft     = hardwareMap.get(DcMotorEx.class,"backLeft"        );
        DcMotorEx bRight    = hardwareMap.get(DcMotorEx.class,"backRight"       );
        DcMotorEx fLeft     = hardwareMap.get(DcMotorEx.class,"frontLeft"       );
        DcMotorEx fRight    = hardwareMap.get(DcMotorEx.class,"frontRight"      );

        bLeft.setVelocity(leftVelocity * scale);
        bRight.setVelocity(rightVelocity * scale);
        fLeft.setVelocity(leftVelocity * scale);
        fRight.setVelocity(rightVelocity * scale);

        bLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        sleep(durationMS);
//
//        bLeft.setVelocity(0);
//        bRight.setVelocity(0);
//        fLeft.setVelocity(0);
//        fRight.setVelocity(0);

//        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runStraight(double DistanceCM, double speed) {
        DcMotorEx bLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx bRight = hardwareMap.get(DcMotorEx.class, "backRight");
        DcMotorEx fLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx fRight = hardwareMap.get(DcMotorEx.class, "frontRight");

        int target = CMtoTicks(DistanceCM);
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        double angle = angles.firstAngle;

        if (opModeIsActive()) {
            bLeft.setTargetPosition(target);
            bRight.setTargetPosition(target);
            fLeft.setTargetPosition(target);
            fRight.setTargetPosition(target);

            bLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            bRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            fLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            fRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            bLeft.setPower(speed);
            bRight.setPower(speed);
            fLeft.setPower(speed);
            fRight.setPower(speed);

            while (opModeIsActive() &&
                    (bLeft.isBusy() && bRight.isBusy() && fLeft.isBusy() && fRight.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (DistanceCM < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                bLeft.setPower(leftSpeed);
                fLeft.setPower(leftSpeed);
                bRight.setPower(rightSpeed);
                fRight.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Error/Steer", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target Position", "Left: %d, Right: %d", target, target);
                telemetry.addData("Current Position", "bL: %d, fL: %d, bR: %d, fR: %d",
                        bLeft.getCurrentPosition(), fLeft.getCurrentPosition(), bRight.getCurrentPosition(), fRight.getCurrentPosition());
                telemetry.addData("Speed", "Left: %5.2f, Right: %5.2f", leftSpeed, rightSpeed);
                telemetry.addData("Straight line:", "In Progress");
                telemetry.update();
            }

            // Stop all motion;
            bLeft.setPower(0);
            fLeft.setPower(0);
            bRight.setPower(0);
            fRight.setPower(0);

            // Turn off RUN_TO_POSITION
            bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData("Straight line:", "Done");
            telemetry.update();
        }
    }//good

    public void og_runStraight(double DistanceCM, double speed) {
        DcMotorEx bLeft     = hardwareMap.get(DcMotorEx.class,"backLeft"        );
        DcMotorEx bRight    = hardwareMap.get(DcMotorEx.class,"backRight"       );
        DcMotorEx fLeft     = hardwareMap.get(DcMotorEx.class,"frontLeft"       );
        DcMotorEx fRight    = hardwareMap.get(DcMotorEx.class,"frontRight"      );

        int target = CMtoTicks(DistanceCM);

        if (opModeIsActive()){
            bLeft.setTargetPosition(target);
            bRight.setTargetPosition(target);
            fLeft.setTargetPosition(target);
            fRight.setTargetPosition(target);

            bLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            bRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            fLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            fRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            bLeft.setPower(speed);
            bRight.setPower(speed);
            fLeft.setPower(speed);
            fRight.setPower(speed);

            while (opModeIsActive() && (bLeft.isBusy() || bRight.isBusy() || fLeft.isBusy() || fRight.isBusy())) {
                telemetry.addData("Target Position", "%d", target);
                telemetry.addData("Current Position", "bL: %d, fL: %d, bR: %d, fR: %d",
                        bLeft.getCurrentPosition(), fLeft.getCurrentPosition(), bRight.getCurrentPosition(), fRight.getCurrentPosition());
                telemetry.addData("Straight line:", "In Progress");
                telemetry.update();
            }

            // Stop all motion;
            bLeft.setPower(0);
            fLeft.setPower(0);
            bRight.setPower(0);
            fRight.setPower(0);

            // Turn off RUN_TO_POSITION
            bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData("Straight line:", "Done");
            telemetry.update();
        }
    }

    public void newnew_arcLeft(double Theta, double radiusCM) { //uses outer and inner radii
        DcMotorEx bLeft     = hardwareMap.get(DcMotorEx.class,"backLeft"        );
        DcMotorEx bRight    = hardwareMap.get(DcMotorEx.class,"backRight"       );
        DcMotorEx fLeft     = hardwareMap.get(DcMotorEx.class,"frontLeft"       );
        DcMotorEx fRight    = hardwareMap.get(DcMotorEx.class,"frontRight"      );

        double innerRadiusInches = radiusCM * .394;
        double innerArcLengthInches = Theta * (Math.PI/180) * innerRadiusInches;
        double innerArcLengthCM = innerArcLengthInches * 2.54;

        double outerRadiusInches = (radiusCM + robotRadius) * .394;
        double outerArcLengthInches = Theta * (Math.PI/180) * outerRadiusInches;
        double outerArcLengthCM = outerArcLengthInches * 2.54;

        int innerTarget = CMtoTicks(innerArcLengthCM);
        int outerTarget = CMtoTicks(outerArcLengthCM);

        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        if (opModeIsActive()) {
            bLeft.setTargetPosition(innerTarget);
            fLeft.setTargetPosition(innerTarget);
            bRight.setTargetPosition(outerTarget);
            fRight.setTargetPosition(outerTarget);

            bLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            bRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            fLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            fRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            // start motion.
            double speed = Range.clip(Math.abs(TURN_SPEED), 0.0, 1.0);
            bLeft.setPower(speed);
            bRight.setPower(speed);
            fLeft.setPower(speed);
            fRight.setPower(speed);

            while (opModeIsActive() &&
                    (bLeft.isBusy() && bRight.isBusy() && fLeft.isBusy() && fRight.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(Theta);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (Theta < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                bLeft.setPower(leftSpeed);
                fLeft.setPower(leftSpeed);
                bRight.setPower(rightSpeed);
                fRight.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Error/Steer", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target Position", "Outer(Left): %d, Inner(Right): %d", outerTarget, innerTarget);
                telemetry.addData("Current Outer Position", "bL: %d, fL: %d",
                        bLeft.getCurrentPosition(), fLeft.getCurrentPosition());
                telemetry.addData("Current Inner Position", "bR: %d, fR: %d",
                        bRight.getCurrentPosition(), fRight.getCurrentPosition());
                telemetry.addData("Speed", "Left: %5.2f, Right: %5.2f", leftSpeed, rightSpeed);
                telemetry.addData("Arc:", "In Progress");
                telemetry.update();
            }
            // Stop all motion;
            bLeft.setPower(0);
            fLeft.setPower(0);
            bRight.setPower(0);
            fRight.setPower(0);

            // Turn off RUN_TO_POSITION
            bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData("Arc:", "Done :)");
            telemetry.update();
        }
    } //inner & outer radii w/ speed controls > needs testing

    public void new_arcRight(double Theta, double radiusCM) {//uses out and inner radii
        DcMotorEx bLeft     = hardwareMap.get(DcMotorEx.class,"backLeft"        );
        DcMotorEx bRight    = hardwareMap.get(DcMotorEx.class,"backRight"       );
        DcMotorEx fLeft     = hardwareMap.get(DcMotorEx.class,"frontLeft"       );
        DcMotorEx fRight    = hardwareMap.get(DcMotorEx.class,"frontRight"      );

        double robotRadius = 40.5;

        double innerRadiusInches = radiusCM * .394;
        double innerArcLengthInches = Theta * (Math.PI/180) * innerRadiusInches;
        double innerArcLengthCM = innerArcLengthInches * 2.54;

        double outerRadiusInches = (radiusCM + robotRadius) * .394;
        double outerArcLengthInches = Theta * (Math.PI/180) * outerRadiusInches;
        double outerArcLengthCM = outerArcLengthInches * 2.54;

        int innerTarget = CMtoTicks(innerArcLengthCM);
        int outerTarget = CMtoTicks(outerArcLengthCM);

        if (opModeIsActive()) {
            bLeft.setTargetPosition(outerTarget);
            fLeft.setTargetPosition(outerTarget);
            bRight.setTargetPosition(innerTarget);
            fRight.setTargetPosition(innerTarget);

            bLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            bRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            fLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            fRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            // start motion.
            double speed = Range.clip(Math.abs(TURN_SPEED), 0.0, 1.0);
            bLeft.setPower(speed);
            bRight.setPower(speed);
            fLeft.setPower(speed);
            fRight.setPower(speed);

            while (opModeIsActive() &&
                    (bLeft.isBusy() && bRight.isBusy() && fLeft.isBusy() && fRight.isBusy())) {
                telemetry.addData("Target Position", "Outer(Left): %d, Inner(Right): %d", outerTarget, innerTarget);
                telemetry.addData("Current Outer Position", "bL: %d, fL: %d",
                        bLeft.getCurrentPosition(), fLeft.getCurrentPosition());
                telemetry.addData("Current Inner Position", "bR: %d, fR: %d",
                        bRight.getCurrentPosition(), fRight.getCurrentPosition());
                telemetry.addData("Arc:", "In Progress");
                telemetry.update();
            }
            // Stop all motion;
            bLeft.setPower(0);
            fLeft.setPower(0);
            bRight.setPower(0);
            fRight.setPower(0);

            // Turn off RUN_TO_POSITION
            bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData("Arc:", "Done :)");
            telemetry.update();
        }
    } //inner & outer radii w/o speed controls > fail

    public void arcLeft(double Theta, double radiusCM) { //only runs right motors (arc degree of bRight only (for now), radius of circle arc 0f bRight only (for now))
        DcMotorEx bLeft     = hardwareMap.get(DcMotorEx.class,"backLeft"        );
        DcMotorEx bRight    = hardwareMap.get(DcMotorEx.class,"backRight"       );
        DcMotorEx fLeft     = hardwareMap.get(DcMotorEx.class,"frontLeft"       );
        DcMotorEx fRight    = hardwareMap.get(DcMotorEx.class,"frontRight"      );

        double radiusInches = (radiusCM + robotRadius) * .394;
        double arcLengthInches = Theta * (Math.PI/180) * radiusInches;
        double arcLengthCM = arcLengthInches * 2.54;
        int target = CMtoTicks(arcLengthCM);

        if (opModeIsActive()) {
            bRight.setTargetPosition(target);
            fRight.setTargetPosition(target);

            bRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            fRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            // start motion.
            bRight.setPower(ARC_SPEED);
            fRight.setPower(ARC_SPEED);

            while (opModeIsActive() && (bRight.isBusy() || fRight.isBusy())) {
                // Display drive status for the driver.
                telemetry.addData("Target Position", "%d", target);
                telemetry.addData("Current Position", "bR: %d, fR: %d",
                        bRight.getCurrentPosition(), fRight.getCurrentPosition());
                telemetry.addData("Speed", "bR: %5.2f, fR: %5.2f", bRight.getPower(), fRight.getPower());
                telemetry.addData("Arc:", "In Progress");
                telemetry.update();
            }
            // Stop all motion;
            bLeft.setPower(0);
            fLeft.setPower(0);
            bRight.setPower(0);
            fRight.setPower(0);

            // Turn off RUN_TO_POSITION
            bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData("Arc:", "Done :)");
            telemetry.update();
        }
    } //outer radii only

    public void arcRight(double Theta, double radiusCM) { //only runs left motors (arc degree of bLeft only (for now), radius of circle arc 0f bLeft only (for now))
        DcMotorEx bLeft     = hardwareMap.get(DcMotorEx.class,"backLeft"        );
        DcMotorEx bRight    = hardwareMap.get(DcMotorEx.class,"backRight"       );
        DcMotorEx fLeft     = hardwareMap.get(DcMotorEx.class,"frontLeft"       );
        DcMotorEx fRight    = hardwareMap.get(DcMotorEx.class,"frontRight"      );

        double radiusInches = (radiusCM + robotRadius) * .394;
        double arcLengthInches = Theta * (Math.PI/180) * radiusInches;
        double arcLengthCM = arcLengthInches * 2.54;
        int target = CMtoTicks(arcLengthCM);

        if (opModeIsActive()) {
            bLeft.setTargetPosition(target);
            fLeft.setTargetPosition(target);

            bLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            fLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            // start motion.
            bLeft.setPower(ARC_SPEED);
            fLeft.setPower(ARC_SPEED);

            while (opModeIsActive() && (bLeft.isBusy() || fLeft.isBusy())) {
                // Display drive status for the driver.
                telemetry.addData("Target Position", "%d", target);
                telemetry.addData("Current Position", "bL: %d, fL: %d",
                        bLeft.getCurrentPosition(), fLeft.getCurrentPosition());
                telemetry.addData("Speed", "bL: %5.2f, fL: %5.2f", bLeft.getPower(), fLeft.getPower());
                telemetry.addData("Arc:", "In Progress");
                telemetry.update();
            }
            // Stop all motion;
            bLeft.setPower(0);
            fLeft.setPower(0);
            bRight.setPower(0);
            fRight.setPower(0);

            // Turn off RUN_TO_POSITION
            bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData("Arc:", "Done :)");
            telemetry.update();
        }
    } //outer radii only

    public void rotateLeft(double Theta) {
        DcMotorEx bLeft     = hardwareMap.get(DcMotorEx.class,"backLeft"        );
        DcMotorEx bRight    = hardwareMap.get(DcMotorEx.class,"backRight"       );
        DcMotorEx fLeft     = hardwareMap.get(DcMotorEx.class,"frontLeft"       );
        DcMotorEx fRight    = hardwareMap.get(DcMotorEx.class,"frontRight"      );

        double robotRadiusCM = 24.3;
        double radiusInches = robotRadiusCM * .394;
        double arcLengthInches = (Theta + (.1*Theta)) * (Math.PI/180) * radiusInches;
        double arcLengthCM = arcLengthInches * 2.54;
        int rightTarget = CMtoTicks(arcLengthCM);
        int leftTarget = -rightTarget;

        bRight.setTargetPosition(rightTarget);
        fRight.setTargetPosition(rightTarget);
        bLeft.setTargetPosition(leftTarget);
        fLeft.setTargetPosition(leftTarget);

        bRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        fRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        bLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        fLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        bRight.setVelocity(1000);
        fRight.setVelocity(1000);
        bLeft.setVelocity(1000);
        fLeft.setVelocity(1000);

        boolean done = false;

        while (!done) {
            if ((Theta > 0) & (bRight.getCurrentPosition() > rightTarget & bLeft.getCurrentPosition() < leftTarget ) | (Theta < 0) & (bRight.getCurrentPosition() < rightTarget & bLeft.getCurrentPosition() > leftTarget )) {
                bRight.setVelocity(0);
                fRight.setVelocity(0);
                bLeft.setVelocity(0);
                fLeft.setVelocity(0);

                bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                telemetry.addData("Arc:", "Done");
//                telemetry.update();

                done = true;
            }
            telemetry.addData("Arc:", "In Progress");
//            telemetry.update();
        }
    } // NEED TO ADD OPMODE CONDITIONALS > see gyro turn

    public void rotateRight(double Theta) {
        DcMotorEx bLeft     = hardwareMap.get(DcMotorEx.class,"backLeft"        );
        DcMotorEx bRight    = hardwareMap.get(DcMotorEx.class,"backRight"       );
        DcMotorEx fLeft     = hardwareMap.get(DcMotorEx.class,"frontLeft"       );
        DcMotorEx fRight    = hardwareMap.get(DcMotorEx.class,"frontRight"      );

        double robotRadiusCM = 24.3;
        double radiusInches = robotRadiusCM * .394;
        double arcLengthInches = (Theta + (.1*Theta)) * (Math.PI/180) * radiusInches;
        double arcLengthCM = arcLengthInches * 2.54;
        int leftTarget = CMtoTicks(arcLengthCM);
        int rightTarget = -leftTarget;

        bRight.setTargetPosition(rightTarget);
        fRight.setTargetPosition(rightTarget);
        bLeft.setTargetPosition(leftTarget);
        fLeft.setTargetPosition(leftTarget);

        bRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        fRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        bLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        fLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        bRight.setVelocity(1000);
        fRight.setVelocity(1000);
        bLeft.setVelocity(1000);
        fLeft.setVelocity(1000);

        boolean done = false;

        while (!done) {
            if ((Theta > 0) & (bRight.getCurrentPosition() < rightTarget & bLeft.getCurrentPosition() > leftTarget ) | (Theta < 0) & (bRight.getCurrentPosition() > rightTarget & bLeft.getCurrentPosition() < leftTarget )) {
                bRight.setVelocity(0);
                fRight.setVelocity(0);
                bLeft.setVelocity(0);
                fLeft.setVelocity(0);

                bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                telemetry.addData("Arc:", "Done");
//                telemetry.update();

                done = true;
            }
            telemetry.addData("Arc:", "In Progress");
//            telemetry.update();
        }
    } // ^

    public int CMtoTicks(double DistanceCM){
        return (int) (DistanceCM * 23.6);
    }

    public void gyroTurn (double angle, double speed) {
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, -angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    } // (rotate) should be good to go

    public void gyroHold(double angle, double speed, double holdTime) {
        DcMotorEx bLeft     = hardwareMap.get(DcMotorEx.class,"backLeft"        );
        DcMotorEx bRight    = hardwareMap.get(DcMotorEx.class,"backRight"       );
        DcMotorEx fLeft     = hardwareMap.get(DcMotorEx.class,"frontLeft"       );
        DcMotorEx fRight    = hardwareMap.get(DcMotorEx.class,"frontRight"      );

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        bLeft.setPower(0);
        fLeft.setPower(0);
        bRight.setPower(0);
        fRight.setPower(0);
    }//basically a second check to make sure the
                                                                            //robot is at the correct heading > should// be good to go
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    boolean onHeading(double speed, double angle, double PCoeff) {
        DcMotorEx bLeft     = hardwareMap.get(DcMotorEx.class,"backLeft"        );
        DcMotorEx bRight    = hardwareMap.get(DcMotorEx.class,"backRight"       );
        DcMotorEx fLeft     = hardwareMap.get(DcMotorEx.class,"frontLeft"       );
        DcMotorEx fRight    = hardwareMap.get(DcMotorEx.class,"frontRight"      );

        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        bLeft.setPower(leftSpeed);
        fLeft.setPower(leftSpeed);
        bRight.setPower(rightSpeed);
        fRight.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Error/Steer", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed", "Left: %5.2f, Right: %5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    public void moveArm(int shoulderPosition, int elbowPosition) {
        DcMotorEx elbow     = hardwareMap.get(DcMotorEx.class,"elbowArmMotor"   );
        DcMotorEx shoulder  = hardwareMap.get(DcMotorEx.class,"shoulderArmMotor");

        double shoulderPower;
        double elbowPower;

        if ((shoulder.getCurrentPosition() == shoulderLow && elbow.getCurrentPosition() == elbowLow)
            && (shoulderPosition == shoulderPickup && elbowPosition == elbowPickup)) {
            shoulderPower = .4;
            elbowPower = .35;
        } else {
            shoulderPower = .5;
            elbowPower = .8;
        }

        if (opModeIsActive()) {
            shoulder.setTargetPosition(shoulderPosition);
            elbow.setTargetPosition(elbowPosition);

            shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            shoulder.setPower(shoulderPower);
            elbow.setPower(elbowPower);

            while (opModeIsActive() && (elbow.isBusy() || shoulder.isBusy())) {
                // Display drive status for the driver.
                telemetry.addData("Arm Position Change:", "In Progress");
                telemetry.update();
            }
            telemetry.addData("Arm Position Change:", "Done :)");
            telemetry.update();
        }

    }

    public void intake(){
        rightIntake = hardwareMap.get(CRServo.class, "RightServo");
        leftIntake = hardwareMap.get(CRServo.class, "LeftServo");

        leftIntake.setPower(1);
        rightIntake.setPower(1);

//        sleep(???);

        leftIntake.setPower(0);
        rightIntake.setPower(0);
    }

    public void output(){
        rightIntake = hardwareMap.get(CRServo.class, "RightServo");
        leftIntake = hardwareMap.get(CRServo.class, "LeftServo");

//        sleep(???);

        leftIntake.setPower(0);
        rightIntake.setPower(0);
    }

    public void brake(long durationMS) {
        DcMotor bLeft     = hardwareMap.get(DcMotorEx.class,"backLeft"        );
        DcMotor bRight    = hardwareMap.get(DcMotorEx.class,"backRight"       );
        DcMotor fLeft     = hardwareMap.get(DcMotorEx.class,"frontLeft"       );
        DcMotor fRight    = hardwareMap.get(DcMotorEx.class,"frontRight"      );

        bLeft.setPower(0);
        bRight.setPower(0);
        fLeft.setPower(0);
        fRight.setPower(0);
        sleep(durationMS);
    }

    public void run_redWarehouse(){

    }

    public void run_redCarousel(){

    }

    public void run_blueWarehouse(){
        new_arcRight(35, radiusHubtoRightSide); // arc towards hub
        gyroHold(35, ARC_SPEED, .5);
//        arcRight(35, 84.405 - robotRadius); //needs opmode conditionals
//        gyroHold(35, ARC_SPEED, .5);
        if (topLevel){
            //arm to top preset
            moveArm(shoulderTopMid, elbowTop);
        }
        else if (middleLevel){
            //arm to middle preset
            moveArm(shoulderTopMid, elbowMid);
        }
        else if (bottomLevel){
            moveArm(shoulderBot, elbowBot); //NEED A BOT POSITION //arm to bottom preset
        } //NEED A BOT POSITION
        else {
            moveArm(shoulderTopMid, elbowTop);
        }
        runStraight(40, DRIVE_SPEED); //forward to hub
        output();//need allotted time necessary for output //release cargo
        runStraight(-40, DRIVE_SPEED); //back away from hub
        moveArm(shoulderLow, elbowLow); //move arm down
        gyroTurn(-90, TURN_SPEED); //rotate left to warehouse
//        rotateLeft(125);
        gyroHold(TURN_SPEED, -90, .25);
        og_runStraight(125, DRIVE_SPEED); //forward into warehouse
        moveArm(shoulderPickup, elbowPickup); //move arm to pickup position
    }

    public void run_blueCarousel(){

    }

    void composeTelemetry() {
        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//            gravity  = imu.getGravity();
        }
        });

//        telemetry.addLine()
//                .addData("status", new Func<String>() {
//                    @Override public String value() {
//                        return imu.getSystemStatus().toShortString();
//                    }
//                })
//                .addData("calib", new Func<String>() {
//                    @Override public String value() {
//                        return imu.getCalibrationStatus().toString();
//                    }
//                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

//        telemetry.addLine()
//                .addData("grvty", new Func<String>() {
//                    @Override public String value() {
//                        return gravity.toString();
//                    }
//                })
//                .addData("mag", new Func<String>() {
//                    @Override public String value() {
//                        return String.format(Locale.getDefault(), "%.3f",
//                                Math.sqrt(gravity.xAccel*gravity.xAccel
//                                        + gravity.yAccel*gravity.yAccel
//                                        + gravity.zAccel*gravity.zAccel));
//                    }
//                });
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
