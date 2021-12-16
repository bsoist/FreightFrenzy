/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Early Drive", group="--")
//@Disabled
public class BasicOpMode_Linear extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor bLeft = null;
    private DcMotor bRight = null;
    private DcMotor fLeft = null;
    private DcMotor fRight = null;
    private DcMotor elbow    = null;
    private DcMotor shoulder = null;
    private Servo   claw     = null;
    private DcMotor ttMotor = null;

    /**
     * Manages toggling an action.
     *
     * Call checkToggleStatus once every loop to determine whether a full button press has
     * occurred or not.
     */
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        DcMotorEx bLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx bRight = hardwareMap.get(DcMotorEx.class, "backRight");
        DcMotorEx fLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx fRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        elbow    = hardwareMap.get(DcMotor.class,   "elbowArmMotor");
        shoulder = hardwareMap.get(DcMotor.class,"shoulderArmMotor");
        claw     = hardwareMap.get(Servo.  class,            "claw");
        ttMotor     = hardwareMap.get(DcMotor.class,"turnTableMotor");

        bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elbow.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotor.Direction.REVERSE);
        fRight.setDirection(DcMotor.Direction.REVERSE);

        double armSpeed = .75;
        claw.setPosition(-1.0);
        boolean elbowBrake = false;
        boolean shoulderBrake = false;
        boolean turnTable = false;


        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //claw controls
            if (gamepad2.a) {
                claw.setPosition( 1.0);
            }

            else if (gamepad2.y) {
                claw.setPosition(-1.0);
            }
            //turn table motor controls
            if (gamepad2.b & !turnTable) {
                turnTable = true;
                ttMotor.setPower(1);
            }
            else if (gamepad2.b & turnTable) {
                ttMotor.setPower((0));
                turnTable = false;
            }

            int elbowCurrentPosition    = elbow   .getCurrentPosition();
            int shoulderCurrentPosition = shoulder.getCurrentPosition();
            int bLeftCP                 = bLeft   .getCurrentPosition();
            int bRightCP                = bRight  .getCurrentPosition();
            int fLeftCP                 = fLeft   .getCurrentPosition();
            int fRightCP                = fRight  .getCurrentPosition();

//            double leftPower;
//            double rightPower;
            double x =  gamepad1.right_stick_x;
            double y =  -gamepad1.right_stick_y;
            double motorVelocity = -gamepad1.left_stick_y * 2500;
            double leftDirection = y + x;
            double rightDirection = y - x;
            double leftVelocity   = leftDirection * motorVelocity;
            double rightVelocity   = rightDirection * motorVelocity;
//            double speed = Range.clip(gamepad1.left_stick_y, -1.0, 0)
//            leftPower    = Range.clip(leftDirection * speed, -1.0, 1.0);
//            rightPower   = Range.clip(rightDirection * speed, -1.0, 1.0);

            double elbowPower    = Range.clip(gamepad2.left_stick_y , -1.0, 1.0) * armSpeed;
            double shoulderPower = Range.clip(gamepad2.right_stick_y, -1.0, 1.0) * armSpeed;

//            bLeft.setPower(leftPower);
//            bRight.setPower(rightPower);
//            fLeft.setPower((leftPower));
//            fRight.setPower(rightPower);

            //DIY ZeroPowerBehavior.Brake
//            if (leftVelocity == 0) {
//                bLeft.setTargetPosition(bLeftCP);
//                fLeft.setTargetPosition(fLeftCP);
//                bLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                fLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }
//            else {
//                bLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            }
//
//            if (rightVelocity == 0) {
//                bRight.setTargetPosition(bRightCP);
//                fRight.setTargetPosition(fRightCP);
//                bRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                fRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }
//            else {
//                bRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                fRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            }
            bLeft.setVelocity(leftVelocity);
            fLeft.setVelocity(leftVelocity);
            bRight.setVelocity(rightVelocity);
            fRight.setVelocity(rightVelocity);

            //elbow brake conditions
            if (gamepad2.left_stick_button & !elbowBrake) {
                elbow.setTargetPosition(elbowCurrentPosition);
                elbowBrake = true;

            }
            else if (gamepad2.left_stick_button & elbowBrake) {
                elbowBrake = false;
            }
            //shoulder brake conditions
            if (gamepad2.right_stick_button & !shoulderBrake) {
                shoulder.setTargetPosition(shoulderCurrentPosition);
                shoulderBrake = true;
            }
            else if (gamepad2.right_stick_button & shoulderBrake) {
                shoulderBrake = false;
            }

            //the brakes themselves
            if (elbowBrake) {
                elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else {
                elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                elbow.setPower(elbowPower);
            }

            if (shoulderBrake) {
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else {
                shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                shoulder.setPower(shoulderPower);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Base Velocity", "(%.2f)", motorVelocity);
            telemetry.addData("Motor Power",
                    "back left (%.2f), back right (%.2f), front left (%.2f), front right (%.2f)",
                            bLeft.getPower(), bRight.getPower(), fLeft.getPower(),  fRight.getPower());
            telemetry.addData("Theoretical Velocity", "left (%.2f), right (%.2f)", leftVelocity, rightVelocity);
            telemetry.addData("Motor Direction", "left (%.2f), right (%.2f)", leftDirection, rightDirection);
            telemetry.addData("Claw"          , "Position: (%.2f)"             , claw.getPosition()              );
            telemetry.addData("Shoulder"      , "Power: (%.2f), Position: (%d), Target Position: (%d), Brake: (%b)", shoulderPower, shoulderCurrentPosition, shoulder.getTargetPosition(), shoulderBrake);
            telemetry.addData("Elbow"         , "Power: (%.2f), Position: (%d), Target Position: (%d), Brake: (%b)", elbowPower, elbowCurrentPosition, elbow.getTargetPosition(), elbowBrake);

            telemetry.update();
        }
    }
}
