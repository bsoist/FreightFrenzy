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

package org.firstinspires.ftc.teamcode.drivetesting;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

class Toggle
{
    /**
     * Holds all the states that a toggle can be in. When pressing a button, there are 3 states:
     * 1. Not begun
     * 2. In progress
     * 3. Complete
     *
     * If you're checking a button press using a conventional on/off check and using it to
     * flip a boolean, then you'll flip once for every time the button is held and the
     * loop iterates.
     */
    enum Status
    {
        NOT_BEGUN ,
        IN_PROGRESS ,
        COMPLETE
    }


    private Status _status = Status.NOT_BEGUN;      // Current status of the toggle


    /**
     *  Monitors and adjusts the toggle value based on previous toggle values and the
     *  state of the boolean passed in.
     */
    public Status status(boolean buttonStatus)
    {
        // If the button is being held
        if(buttonStatus && _status == Status.NOT_BEGUN)
            _status = Status.IN_PROGRESS;

            // If the button is not being pressed and the toggle was in progress
        else if(!buttonStatus && _status == Status.IN_PROGRESS)
            _status = Status.COMPLETE;

            // If the toggle is finished
        else if(_status == Status.COMPLETE)
            _status = Status.NOT_BEGUN;

        return _status;
    }
}
@TeleOp(name="Green Bot Arm Test", group="--")
@Disabled
public class greenBotTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor bLeft    = null;
    private DcMotor bRight   = null;
    private DcMotor fLeft    = null;
    private DcMotor fRight   = null;
    private DcMotor elbow    = null;
    private DcMotor shoulder = null;
    private Servo   claw     = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        bLeft    = hardwareMap.get(DcMotor.class,        "rearLeft");
        bRight   = hardwareMap.get(DcMotor.class,       "rearRight");
        fLeft    = hardwareMap.get(DcMotor.class,       "frontLeft");
        fRight   = hardwareMap.get(DcMotor.class,      "frontRight");
        elbow    = hardwareMap.get(DcMotor.class,   "elbowArmMotor");
        shoulder = hardwareMap.get(DcMotor.class,"shoulderArmMotor");
        claw     = hardwareMap.get(Servo.  class,            "claw");



        bRight.setDirection(DcMotor.Direction.REVERSE);
        fRight.setDirection(DcMotor.Direction.REVERSE);
        elbow .setDirection(DcMotor.Direction.REVERSE);

        //elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double armSpeed = 1;
        claw.setPosition(-1.0);
        boolean elbowBrake = false;
        boolean shoulderBrake = false;

        Toggle eBrake = new Toggle();    // Slows down drivetrain when on
        Toggle sBrake = new Toggle();    // Slows down drivetrain when on

        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double x =   -gamepad1.right_stick_x;
            double y =   gamepad1.right_stick_y;
            double speed = Range.clip(gamepad1.left_stick_y, -1.0, 0);

            int elbowCurrentPosition    =    elbow.getCurrentPosition();
            int shoulderCurrentPosition = shoulder.getCurrentPosition();




            if (gamepad2.a) {
                claw.setPosition( 1.0);
            }
            else if (gamepad2.y) {
                claw.setPosition(-1.0);
            }

            //left open, right closed

            double leftDirection =  y + x;
            double rightDirection = y - x;

            double leftPower     = Range.clip(leftDirection         , -1.0, 1.0) *    speed;
            double rightPower    = Range.clip(rightDirection        , -1.0, 1.0) *    speed;
            double elbowPower    = Range.clip(gamepad2.left_stick_y , -1.0, 1.0) * armSpeed;
            double shoulderPower = Range.clip(gamepad2.right_stick_y, -1.0, 1.0) * armSpeed;

            // Send calculated power to wheels
            bLeft   .setPower    (leftPower);
            bRight  .setPower   (rightPower);
            fLeft   .setPower    (leftPower);
            fRight  .setPower   (rightPower);

            //if left stick button has been toggled
//            if(eBrake.status(gamepad2.left_stick_button) == Toggle.Status.COMPLETE & !elbowBrake)
//                elbowBrake = true;
//            if(eBrake.status(gamepad2.left_stick_button) == Toggle.Status.COMPLETE & elbowBrake)
//                elbowBrake = false;

            if (elbowBrake) {
                int elbowBrakePosition = elbow.getCurrentPosition();
                elbow.setTargetPosition(elbowBrakePosition);
                elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else {
                elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                elbow.setPower(elbowPower);
            }

            //if right stick button has been toggled
//            if(sBrake.status(gamepad2.right_stick_button) == Toggle.Status.COMPLETE & !shoulderBrake)
//                shoulderBrake = true;
//            if(sBrake.status(gamepad2.right_stick_button) == Toggle.Status.COMPLETE & shoulderBrake)
//                shoulderBrake = false;

            if (shoulderBrake) {
                int shoulderBrakePosition = shoulder.getCurrentPosition();
                shoulder.setTargetPosition(shoulderBrakePosition);
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else {
                shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                shoulder.setPower(shoulderPower);
            }


            if (gamepad2.left_stick_button & !elbowBrake) {
                    elbowBrake = true;
                }
            else if (gamepad2.left_stick_button & elbowBrake) {
                elbowBrake = false;
            }

            if (gamepad2.right_stick_button & !shoulderBrake) {
                    shoulderBrake = true;
                }
             else if (gamepad2.right_stick_button & shoulderBrake) {
                shoulderBrake = false;
            }

            // Returning Telemetry Data
            telemetry.addData("Status"        ,  "Run Time: " + runtime.toString());
            telemetry.addData("Intended Power", "left (%.2f), right (%.2f)"    , leftDirection, rightDirection   );
            telemetry.addData("Chassis Motors", "left (%.2f), right (%.2f)"    , leftPower, rightPower           );
            telemetry.addData("Claw"          , "Position: (%.2f)"             , claw.getPosition()              );
            telemetry.addData("Shoulder"      , "Power: (%.2f), Position: (%d), Target Position: (%d), Brake: (%b)", shoulderPower, shoulderCurrentPosition, shoulder.getTargetPosition(), shoulderBrake);
            telemetry.addData("Elbow"         , "Power: (%.2f), Position: (%d), Target Position: (%d), Brake: (%b)", elbowPower, elbowCurrentPosition, elbow.getTargetPosition(), elbowBrake);
            telemetry.update();
        }
    }
}
