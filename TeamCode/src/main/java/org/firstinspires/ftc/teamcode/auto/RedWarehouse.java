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
package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Freight Frenzy game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Red Warehouse", group = "-")
// @Disabled
public class RedWarehouse extends LinearOpMode {
  /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
   * the following 4 detectable objects
   *  0: Ball,
   *  1: Cube,
   *  2: Duck,
   *  3: Marker (duck location tape marker)
   *
   *  Two additional model assets are available which only contain a subset of the objects:
   *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
   *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
   */
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
      "Ball",
      "Cube",
      "Duck",
      "Marker"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "ASYVnwL/////AAABmWrpGTWsBERHmzsoqdXt1m9+EuzfuJTnTl/KNp3SCy/HnqRDeol4cGWOVd85vtLGg2wokTI1g1nhXrRN2C7ygXKdnaxsVlMsxKsRxdo1KqGPaqIDVFlTAvlonb9GxC1wQrtIIUCdGioLB+/fMSGehrM/Mdv9+vsiezia4kbqJo1uAZIeQPZSG44qm2tED+Qt85CaQVGx4wSa82+hzwLL8sGCmccaOGBhgL4V3XNU5McKdLEANZqJczeh819pEdXEzxBqTHsu131XdQMD0F1U7A4BuhHtKc3VFMQ5pguZzdX7iOp/hvPDTaqKBHQxLrK8uCxnwa56FIFZ2/jUtY90eK83K3aTS2qoFJpjSfALIn+L";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    private DcMotor bLeft = null;
    private DcMotor bRight = null;
    private DcMotor fLeft = null;
    private DcMotor fRight = null;
    private DcMotor elbow = null;
    private DcMotor shoulder = null;
    private Servo   claw     = null;
    private DcMotor ttMotor = null;

    //double leftPower = 0;
    //double rightPower = 0;
//    double speed = .4;
//    long scale = 4;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        DcMotorEx bLeft     = hardwareMap.get(DcMotorEx.class,"backLeft"        );
        DcMotorEx bRight    = hardwareMap.get(DcMotorEx.class,"backRight"       );
        DcMotorEx fLeft     = hardwareMap.get(DcMotorEx.class,"frontLeft"       );
        DcMotorEx fRight    = hardwareMap.get(DcMotorEx.class,"frontRight"      );
        DcMotor elbow     = hardwareMap.get(DcMotorEx.class,"elbowArmMotor"   );
        DcMotor shoulder  = hardwareMap.get(DcMotorEx.class,"shoulderArmMotor");
        claw                = hardwareMap.get(Servo.class,    "claw"            );
        ttMotor             = hardwareMap.get(DcMotorEx.class,"turnTableMotor"  );

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

        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        boolean bottomLevel = false;
        boolean middleLevel = false;
        boolean topLevel = false;

        String position[] = {"left", "middle", "right"};

        String drive = position[2];

        int Sdelta = 1504;
        int Edelta = 394;

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.5, 16.0/9.0);
        }

//        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        claw.setPosition(1);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());



                            if (recognition.getLeft() < 280) {
                                drive = position[2]; // middle > middle of shipping hub
                            } else if (recognition.getLeft() > 280) {
                                drive = position[1]; // right > top of shipping hub
                            }
                            i++;
                            sleep(5000);
                        }

                    }
                    else { //tenorflow sees nothing > left > bottom of shipping hub
                        drive = position[0];
                    }

                }
                telemetry.addData("Barcode Position", drive);
                telemetry.update();

                if(drive == position[0]){
                    bottomLevel = true;
                    middleLevel = false;
                    topLevel = false;
                    telemetry.addData("Target:", "bottom level");
                    telemetry.update();
                }
                else if(drive == position[1]){
                    tfod.deactivate();
                    bottomLevel = false;
                    middleLevel = true;
                    topLevel = false;
                    telemetry.addData("Target:", "middle level");
                    telemetry.update();
                }
                else if(drive == position[2]){
                    tfod.deactivate();
                    bottomLevel = false;
                    middleLevel = false;
                    topLevel = true;
                    telemetry.addData("Target:", "top level");
                    telemetry.update();
                }

                telemetry.addData("Auto:", "In Progress");
                telemetry.update();

                sleep(500);

                arcLeft(30, 84.405);//(arc degree of bLeft to the center of hub, radius of circle arc bLeft to center of hub )

                sleep(5000);

                if (bottomLevel){
                    //set arm to bottom preset
                    shoulder.setTargetPosition(-1937 - Sdelta);
                    elbow.setTargetPosition(75 - Edelta);
                    elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                else if (middleLevel){
                    //set arm to middle preset
                    shoulder.setTargetPosition(0 - Sdelta);
                    elbow.setTargetPosition(240 - Edelta);
                    elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                else if (topLevel){
                    //set arm to top preset
                    shoulder.setTargetPosition(0 - Sdelta);
                    elbow.setTargetPosition(150 - Edelta);
                    elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                else {
                    //set arm to top preset
                    shoulder.setTargetPosition(0 - Sdelta);
                    elbow.setTargetPosition(150 - Edelta);
                    elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                sleep(5000); //stall for the rest of auto`

                if (bottomLevel) {
                    runStraight(40); //approach hub
                }
                else {
                    runStraight(50); // approach hub
                }

                telemetry.addData("Cargo Release:", "Starting");
                telemetry.update();

                if (bottomLevel | middleLevel) {
                    claw.setPosition(.75);
                }
                else {
                    claw.setPosition(.4);
                }

                telemetry.addData("Cargo Release:", "Done");
                telemetry.update();

                runStraight(-10); // back slightly from hub

                sleep(1000);

                arcLeft(-123, 40.05);//(arc degree of bLeft to the center of hub, radius of circle arc bLeft to bRight )
                //rotate toward warehouse

                sleep(6000);

                runStraight(200); // drive into warehouse

                telemetry.addData("Auto:", "Complete :)");
                telemetry.update();
                brake(30000); //stall for the rest of auto

            }
        }
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

    public void runStraight(double DistanceCM) {
        DcMotorEx bLeft     = hardwareMap.get(DcMotorEx.class,"backLeft"        );
        DcMotorEx bRight    = hardwareMap.get(DcMotorEx.class,"backRight"       );
        DcMotorEx fLeft     = hardwareMap.get(DcMotorEx.class,"frontLeft"       );
        DcMotorEx fRight    = hardwareMap.get(DcMotorEx.class,"frontRight"      );

        int target = CMtoTicks(DistanceCM);

        bLeft.setTargetPosition(target);
        bRight.setTargetPosition(target);
        fLeft.setTargetPosition(target);
        fRight.setTargetPosition(target);

        bLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        bRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        fLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        fRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        bLeft.setVelocity(1000);
        bRight.setVelocity(1000);
        fLeft.setVelocity(1000);
        fRight.setVelocity(1000);

        boolean done = false;

        while (!done) {
            if (target < bLeft.getCurrentPosition()) {
                bLeft.setVelocity(0);
                bRight.setVelocity(0);
                fLeft.setVelocity(0);
                fRight.setVelocity(0);

                bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                telemetry.addData("Straight line:", "Done");
                telemetry.update();
                done = true;
            }
            telemetry.addData("Straight line:", "In Progress");
            telemetry.update();
        }
    }

    public void arcRight(double Theta, double radiusCM) { //only runs left motors (arc degree of bLeft only (for now), radius of circle arc 0f bLeft only (for now))
        DcMotorEx bLeft     = hardwareMap.get(DcMotorEx.class,"backLeft"        );
        DcMotorEx bRight    = hardwareMap.get(DcMotorEx.class,"backRight"       );
        DcMotorEx fLeft     = hardwareMap.get(DcMotorEx.class,"frontLeft"       );
        DcMotorEx fRight    = hardwareMap.get(DcMotorEx.class,"frontRight"      );

        double radiusInches = radiusCM * .394;
        double arcLengthInches = Theta * (Math.PI/180) * radiusInches;
        double arcLengthCM = arcLengthInches * 2.54;
        int target = CMtoTicks(arcLengthCM);

        bLeft.setTargetPosition(target);
        fLeft.setTargetPosition(target);

        bLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        fLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        bLeft.setVelocity(1000);
        fLeft.setVelocity(1000);

        boolean done = false;

        while (!done) {
            if (target < bLeft.getCurrentPosition()) {
                bLeft.setVelocity(0);
                fLeft.setVelocity(0);

                bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                telemetry.addData("Arc:", "Done");
                telemetry.update();

                done = true;
            }
            telemetry.addData("Arc:", "In Progress");
            telemetry.update();
        }
    }

    public void arcLeft(double Theta, double radiusCM) { //only runs left motors (arc degree of bLeft only (for now), radius of circle arc 0f bLeft only (for now))
        DcMotorEx bLeft     = hardwareMap.get(DcMotorEx.class,"backLeft"        );
        DcMotorEx bRight    = hardwareMap.get(DcMotorEx.class,"backRight"       );
        DcMotorEx fLeft     = hardwareMap.get(DcMotorEx.class,"frontLeft"       );
        DcMotorEx fRight    = hardwareMap.get(DcMotorEx.class,"frontRight"      );

        double radiusInches = radiusCM * .394;
        double arcLengthInches = Theta * (Math.PI/180) * radiusInches;
        double arcLengthCM = arcLengthInches * 2.54;
        int target = CMtoTicks(arcLengthCM);

        bRight.setTargetPosition(target);
        fRight.setTargetPosition(target);

        bRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        fRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        bRight.setVelocity(1000);
        fRight.setVelocity(1000);

        boolean done = false;

        while (!done) {
            if (target < bRight.getCurrentPosition()) {
                bRight.setVelocity(0);
                fRight.setVelocity(0);

                bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                telemetry.addData("Arc:", "Done");
                telemetry.update();

                done = true;
            }
            telemetry.addData("Arc:", "In Progress");
            telemetry.update();
        }
    }

    public int CMtoTicks(double DistanceCM){
        return (int) (DistanceCM * 23.6);
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

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine. appContext.getPackageName()
     */
    private void initTfod() {
        int tfodMonitorViewId;
        tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
       tfodParameters.minResultConfidence = 0.8f;
       tfodParameters.isModelTensorFlow2 = true;
       tfodParameters.inputSize = 320;
       tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
       tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}
