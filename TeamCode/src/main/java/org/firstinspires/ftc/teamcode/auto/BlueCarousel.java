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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
@Autonomous(name = "Blue Carousel", group = "-")
// @Disabled
public class BlueCarousel extends LinearOpMode {

    private DcMotor bLeft = null;
    private DcMotor bRight = null;
    private DcMotor fLeft = null;
    private DcMotor fRight = null;
    private DcMotor ttMotor = null;


    //double leftPower = 0;
    //double rightPower = 0;
    double speed = .4;
    long scale = 4;

    @Override
    public void runOpMode() {
        bLeft = hardwareMap.get(DcMotor.class, "backLeft");
        bRight = hardwareMap.get(DcMotor.class, "backRight");
        fLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        fRight = hardwareMap.get(DcMotor.class, "frontRight");
        ttMotor = hardwareMap.get(DcMotor  .class,"turnTableMotor");

        bLeft.setDirection(DcMotor.Direction.REVERSE);
        fLeft.setDirection(DcMotor.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);

        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                arcRight(10,20.0025); //arc right into carousel
                brake(1000); //stop
                ttMotor.setPower(1);
                sleep(5000);//turn carousel for 5 secs
                brake(30000);//stall for rest of auto
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

    public void runMotorsVel(double leftVelocity, double rightVelocity, double scale, long durationMS) {
        DcMotorEx bLeft     = hardwareMap.get(DcMotorEx.class,"backLeft"        );
        DcMotorEx bRight    = hardwareMap.get(DcMotorEx.class,"backRight"       );
        DcMotorEx fLeft     = hardwareMap.get(DcMotorEx.class,"frontLeft"       );
        DcMotorEx fRight    = hardwareMap.get(DcMotorEx.class,"frontRight"      );

        bLeft.setVelocity(leftVelocity * scale);
        bRight.setVelocity(rightVelocity * scale);
        fLeft.setVelocity(leftVelocity * scale);
        fRight.setVelocity(rightVelocity * scale);

        sleep(durationMS);

        bLeft.setVelocity(0);
        bRight.setVelocity(0);
        fLeft.setVelocity(0);
        fRight.setVelocity(0);

        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
                done = true;
            }
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
                done = true;
            }
        }
    }

    public int CMtoTicks(double DistanceCM){
        return (int) (DistanceCM * 23.6);
    }

    public void brake(long durationMS) {
        bLeft.setPower(0);
        bRight.setPower(0);
        fLeft.setPower(0);
        fRight.setPower(0);
        sleep(durationMS);
    }
}
