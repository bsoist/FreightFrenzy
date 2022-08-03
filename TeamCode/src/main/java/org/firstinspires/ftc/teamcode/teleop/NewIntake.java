package org.firstinspires.ftc.teamcode.teleop;

import android.graphics.Paint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="NewIntake", group="--")
//@Disabled
public class NewIntake extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor bLeft = null;
    private DcMotor bRight = null;
    private DcMotor fLeft = null;
    private DcMotor fRight = null;
    private DcMotor elbow    = null;
    private DcMotor shoulder = null;
    private Servo   claw     = null;
    private CRServo rightIntake = null;
    private CRServo leftIntake  = null;
    private DcMotor ttMotor = null;
    DigitalChannel magLimit;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        DcMotorEx bLeft     = hardwareMap.get(DcMotorEx.class,"backLeft"        );
        DcMotorEx bRight    = hardwareMap.get(DcMotorEx.class,"backRight"       );
        DcMotorEx fLeft     = hardwareMap.get(DcMotorEx.class,"frontLeft"       );
        DcMotorEx fRight    = hardwareMap.get(DcMotorEx.class,"frontRight"      );
        DcMotorEx elbow     = hardwareMap.get(DcMotorEx.class,"elbowArmMotor"   );
        DcMotorEx shoulder  = hardwareMap.get(DcMotorEx.class,"shoulderArmMotor");
        claw                = hardwareMap.get(Servo    .class,"claw"            );
        ttMotor             = hardwareMap.get(DcMotor  .class,"turnTableMotor"  );
        rightIntake         = hardwareMap.get(CRServo    .class,"RightServo"  );
        leftIntake          = hardwareMap.get(CRServo    .class,"LeftServo"  );
        magLimit = hardwareMap.get(DigitalChannel.class, "magLimitSwitch");

        magLimit.setMode(DigitalChannel.Mode.INPUT);

        bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ttMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bRight.setDirection(DcMotor.Direction.REVERSE);
        fRight.setDirection(DcMotor.Direction.REVERSE);
        leftIntake.setDirection(CRServo.Direction.REVERSE);

        double armSpeed = .75;
        double elbowPower = 0;
        double shoulderPower = 0;
        claw.setPosition(-1.0);
        boolean elbowBrake = false;
        boolean shoulderBrake = false;
        boolean blueturnTable = false;
        boolean redturnTable = false;
        boolean topMidLevel = false;
        boolean pickupLevel = false;
        boolean wobbleLevel = false;
        boolean autoControl = false;
        boolean Cap = false;
        boolean Zero = false;
        boolean topLevel = false;
        boolean midLevel = false;
        boolean botLevel = false;
        int Sdelta = 1504;
        int Edelta = 394;
        boolean calibrate = false;
        boolean reached = false;
        double motorVelocity = 2000;
        boolean output = false;
        boolean intake = false;
        int shoulderTopMidBot = -700;
        int shoulderPickup = -1095;
        int shoulderWobble = -838;
        int shoulderZero = -15;
        int shoulderCap = -866;
        int elbowTop = 283;
        int elbowMid = 190;
        int elbowBot = 73;
        int elbowPickup = 133;
        int elbowWobble = 77;
        int elbowZero = 20;
        int elbowCap = 390;
//            -866, 401 (387)

//        while (!calibrate) {
//            if (!magLimit.getState()){
//                telemetry.addData("MagLimit", "Pressed");
//                telemetry.update();
//                shoulder.setPower(0);
//                calibrate = true;
//            }
//            telemetry.addData("MagLimit", "Not Pressed");
//            telemetry.update();
//            shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            shoulder.setPower(.15);
//        }
//        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            int elbowCurrentPosition    = elbow   .getCurrentPosition();
            int shoulderCurrentPosition = shoulder.getCurrentPosition();

            //claw controls
            if (gamepad2.a) {
                claw.setPosition(1.0);
            }

            if (gamepad2.y) {
                if (autoControl) {
                    claw.setPosition(.4);
                }
                else {
                    claw.setPosition(.2);
                }
            }
            else if (gamepad1.y) {
                claw.setPosition(0);
            }

            ///old turn table motor controls
//            if (gamepad2.b & !redturnTable) {
//                redturnTable = true;
//                ttMotor.setPower(-1);
//            }
//            else if (gamepad2.b & redturnTable) {
//                ttMotor.setPower((0));
//                redturnTable = false;
//            }
//
//            if (gamepad2.x & !blueturnTable) {
//                blueturnTable = true;
//                ttMotor.setPower(1);
//            }
//            else if (gamepad2.x & blueturnTable) {
//                ttMotor.setPower((0));
//                blueturnTable = false;
//            }

            if (gamepad2.dpad_right) { // WOBBLE
                topMidLevel = false;
                wobbleLevel = true;
                pickupLevel = false;
                Zero = false;

                autoControl = true;
                reached = false;
            }
            else if (gamepad2.dpad_up) { // TOP/MID
                topMidLevel = true;
                wobbleLevel = false;
                pickupLevel = false;
                Zero = false;

                autoControl = true;
                reached = false;
            }
            else if (gamepad2.dpad_down) { //  PICKUP
                topMidLevel = false;
                wobbleLevel = false;
                pickupLevel = true;
                Zero = false;

                autoControl = true;
                reached = false;
            }
            else if (gamepad2.right_stick_button && gamepad2.left_stick_button) {
                topMidLevel = false;
                wobbleLevel = false;
                pickupLevel = false;
                Zero = true;

                autoControl = true;
                reached = false;
            }
            else if (gamepad2.dpad_left) { // AUTOCONTROL OFF
                wobbleLevel = false;
                topMidLevel = false;
                pickupLevel = false;
                Zero = false;

                autoControl = false;
                reached = false;
            }

            if (gamepad2.y) {
                topLevel = true;
                midLevel = false;
                botLevel = false;
                Cap = false;
            }
            else if (gamepad2.b) {
                topLevel = false;
                midLevel = true;
                botLevel = false;
                Cap = false;
            }
            else if (gamepad2.a) {
                topLevel = false;
                midLevel = false;
                botLevel = true;
                Cap = false;
            }
            else if (gamepad2.x) {
                topLevel = false;
                midLevel = false;
                botLevel = false;
                Cap = true;
            }

            if (wobbleLevel) { //needs testing
                shoulder.setTargetPosition(shoulderWobble);
                elbow.setTargetPosition(elbowWobble);
            }
            else if (topMidLevel) { //correct
                //set arm to top preset
                if (Cap){
                    shoulder.setTargetPosition(shoulderCap);
                } else if (botLevel){
                    shoulder.setTargetPosition(shoulderWobble);
                } else {
                    shoulder.setTargetPosition(shoulderTopMidBot);
                }

                if (topLevel) {
                    elbow.setTargetPosition(elbowTop);
                } else if (midLevel) {
                    elbow.setTargetPosition(elbowMid);
                } else if (botLevel) {
                    elbow.setTargetPosition(elbowWobble+45);
                } else if (Cap) {
                    elbow.setTargetPosition(elbowCap);
                } else {
                    elbow.setTargetPosition(elbowTop);
                }

            }
            else if (pickupLevel) { // correct
                shoulder.setTargetPosition(shoulderPickup);
                elbow.setTargetPosition(elbowPickup);
            }
            else if (Cap) {
                shoulder.setTargetPosition(shoulderCap);
                elbow.setTargetPosition(elbowCap);
            }
            else if (Zero) {
                shoulder.setTargetPosition(shoulderZero);
                elbow.setTargetPosition(elbowZero);
            }

            if (autoControl) {
                elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (Zero) { elbow.setVelocity(600); }
                else { elbow.setVelocity(2000); }

                if (reached){ shoulder.setVelocity(300); }
                else { //preset power management
                    shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    shoulder.setVelocity(1200);
                    if (shoulderCurrentPosition > shoulder.getTargetPosition()) {
                        if (shoulder.getCurrentPosition() <= shoulder.getTargetPosition()) {
                            reached = true;
                        }
                    }
                    else {
                        if (shoulder.getCurrentPosition() >= shoulder.getTargetPosition()) {
                            reached = true;
                        }
                    }
                }
            } else {
                // (shoulder and elbow brake if autocontrol = false)
                if (shoulderBrake) { // brakes only work if using arm manually
                    shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    shoulder.setVelocity(1800);
                } else {
                    shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    shoulder.setPower(shoulderPower);
                }

                if (elbowBrake) { // brakes only work if using arm manually
                    elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elbow.setVelocity(2500);
                } else {
                    elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    elbow.setPower(elbowPower);
                }
            }


            //driver carousel motor control
            double ttMotorPower = Range.clip(gamepad1.left_stick_x, -1, 1);
            ttMotor.setPower(ttMotorPower);


            double x =  gamepad1.right_stick_x;
            double y =  -gamepad1.right_stick_y;
//            double motorVelocity = -gamepad1.left_stick_y * 2500;

            double leftDirection = y + x;
            double rightDirection = y - x;
            double leftVelocity   = leftDirection * motorVelocity;
            double rightVelocity   = rightDirection * motorVelocity;

            if (gamepad1.right_bumper){ //speed drive
                motorVelocity = 2500;
            }
            if (gamepad1.left_bumper){ //slug drive
                motorVelocity = 750;
            }

            bLeft.setVelocity(leftVelocity);
            fLeft.setVelocity(leftVelocity);
            bRight.setVelocity(rightVelocity);
            fRight.setVelocity(rightVelocity);

            elbowPower = Range.clip(gamepad2.left_stick_y , -1.0, 1.0) * armSpeed;
            shoulderPower = Range.clip(gamepad2.right_stick_y , -1.0, 1.0) * armSpeed;

            if (gamepad2.left_trigger > gamepad2.right_trigger) {
                intake = false;
                output = true;
            }
            else if (gamepad2.right_trigger > gamepad2.left_trigger) {
                intake = true;
                output = false;
            }
            else if (gamepad2.right_trigger > -.2 && gamepad2.left_trigger > -.2) {
                intake = false;
                output = false;
                rightIntake.setPower(0);
                leftIntake.setPower(0);
            }
//            else {
//                intake = false;
//                output = false;
//            }

            double intakePower = gamepad2.right_trigger;
            double outputPower = gamepad2.left_trigger;

            if (intake){
                rightIntake.setPower(intakePower);
                leftIntake.setPower(intakePower);
            }
            if (output){
                rightIntake.setPower(-outputPower);
                leftIntake.setPower(-outputPower);
            }


            if (gamepad2.left_bumper) {
                bLeft.setPower(-1);
                fLeft.setPower(-1);
                bRight.setPower(1);
                fRight.setPower(1);
                sleep(100);
                bLeft.setPower(0);
                fLeft.setPower(0);
                bRight.setPower(0);
                fRight.setPower(0);
            }

            if (gamepad2.right_bumper) {
                bLeft.setPower(1);
                fLeft.setPower(1);
                bRight.setPower(-1);
                fRight.setPower(-1);
                sleep(100);
                bLeft.setPower(0);
                fLeft.setPower(0);
                bRight.setPower(0);
                fRight.setPower(0);
            }


            //elbow brake conditions
            if (gamepad2.x & !elbowBrake) {
                elbow.setTargetPosition(elbowCurrentPosition);
                elbowBrake = true;

            }
            else if (gamepad2.x & elbowBrake) {
                elbowBrake = false;
            }

            //shoulder brake conditions
            if (gamepad2.b & !shoulderBrake) {
                shoulder.setTargetPosition(shoulderCurrentPosition);
                shoulderBrake = true;
            }
            else if (gamepad2.b & shoulderBrake) {
                shoulderBrake = false;
            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status"       , "Run Time: " + runtime.toString());

            telemetry.addData("","----------");


            telemetry.addData("Motor Power",
                    "back left (%.2f), back right (%.2f), front left (%.2f), front right (%.2f)",
                    bLeft.getPower(), bRight.getPower(), fLeft.getPower(),  fRight.getPower());
            telemetry.addData("Base Velocity", "(%.2f)"    , motorVelocity     );
            telemetry.addData("Motor Direction"     , "left (%.2f), right (%.2f)", leftDirection, rightDirection);
            telemetry.addData("Motor Theoretical Velocity", "left (%.2f), right (%.2f)", leftVelocity , rightVelocity );

            telemetry.addData("","----------");

            telemetry.addData("Shoulder",
                    "Power: (%.2f), Position: (%d), Target Position: (%d), Brake: (%b)",
                    shoulderPower, shoulderCurrentPosition, shoulder.getTargetPosition(), shoulderBrake);
            telemetry.addData("Elbow",
                    "Power: (%.2f), Position: (%d), Target Position: (%d), Brake: (%b)",
                    elbowPower   , elbowCurrentPosition   , elbow.getTargetPosition()    , elbowBrake  );

            telemetry.addData("","----------");

            telemetry.addData("Intake",
                    "Left Trig: (%.2f), Right Trig: (%.2f), Intake: (%b) ((%.2f)), Output: (%b) ((%.2f))",
                    gamepad2.left_trigger, gamepad2.right_trigger, intake, intakePower, output, outputPower
            );

            telemetry.addData("","----------");

            telemetry.addData("autoControl"         , "Wobble: (%b), Top/Mid: (%b), Pickup: (%b), autoControl: (%b), Reached: (%b)", wobbleLevel, topMidLevel, pickupLevel, autoControl, reached);


            if (magLimit.getState() == true) {
                telemetry.addData("MagLimit", "False");
            }
            else {
                telemetry.addData("MagLimit", "True");
            }

            telemetry.update();
        }
    }
}