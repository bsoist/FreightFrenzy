package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="HalfPresetDrive", group="--")
//@Disabled
public class HalfPresetDrive extends LinearOpMode {

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
        magLimit = hardwareMap.get(DigitalChannel.class, "magLimitSwitch");

        magLimit.setMode(DigitalChannel.Mode.INPUT);

        bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        bRight.setDirection(DcMotor.Direction.REVERSE);
        fRight.setDirection(DcMotor.Direction.REVERSE);

        elbow.setMode((DcMotor.RunMode.RUN_WITHOUT_ENCODER));

        double armSpeed = .75;
        double elbowPower = 0;
        double shoulderPower = 0;
        claw.setPosition(-1.0);
        boolean elbowBrake = false;
        boolean shoulderBrake = false;
        boolean blueturnTable = false;
        boolean redturnTable = false;
        boolean topMidLevel = false;
        boolean botPickupLevel = false;
        boolean wobbleLevel = false;
        boolean autoControl = false;
        int Sdelta = 1504;
        int Edelta = 394;
        boolean calibrate = false;
        boolean reached = false;

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
                    claw.setPosition(.6);
                }
                else {
                    claw.setPosition(.2);
                }
            }
            else if (gamepad1.y) {
                claw.setPosition(.2);
            }

            ///turn table motor controls
            if (gamepad2.b & !redturnTable) {
                redturnTable = true;
                ttMotor.setPower(-1);
            }
            else if (gamepad2.b & redturnTable) {
                ttMotor.setPower((0));
                redturnTable = false;
            }

            if (gamepad2.x & !blueturnTable) {
                blueturnTable = true;
                ttMotor.setPower(1);
            }
            else if (gamepad2.x & blueturnTable) {
                ttMotor.setPower((0));
                blueturnTable = false;
            }


            if (gamepad2.dpad_up) { // WOBBLE
                wobbleLevel = true;
                topMidLevel = false;
                botPickupLevel = false;

                autoControl = true;
                reached = false;
            }
            else if (gamepad2.dpad_right) { // TOP/MID
                wobbleLevel = false;
                topMidLevel = true;
                botPickupLevel = false;

                autoControl = true;
                reached = false;
            }
            else if (gamepad2.dpad_down) { // BOT/PICKUP
                wobbleLevel = false;
                topMidLevel = false;
                botPickupLevel = true;

                autoControl = true;
                reached = false;
            }
            else if (gamepad2.dpad_left) { // AUTOCONTROL OFF
                wobbleLevel = false;
                topMidLevel = false;
                botPickupLevel = false;
                autoControl = false;
                reached = false;
            }

            if (wobbleLevel) { //needs testing
                shoulder.setTargetPosition(-100);
            }
            else if (topMidLevel) { //correct
                //set arm to top preset
                shoulder.setTargetPosition(0 - Sdelta);
            }
            else if (botPickupLevel) { // correct
                shoulder.setTargetPosition(-1937 - Sdelta);
            }

            if (autoControl) {
                if (reached){
                    shoulder.setVelocity(0);
                }
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

            }

            if (elbowBrake) { // brakes only work if using arm manually
                elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elbow.setVelocity(1000);
            }
            else {
                elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                elbow.setPower(elbowPower);
            }

            double x =  gamepad1.right_stick_x;
            double y =  -gamepad1.right_stick_y;
            double motorVelocity = -gamepad1.left_stick_y * 2500;
            double leftDirection = y + x;
            double rightDirection = y - x;
            double leftVelocity   = leftDirection * motorVelocity;
            double rightVelocity   = rightDirection * motorVelocity;

            bLeft.setVelocity(leftVelocity);
            fLeft.setVelocity(leftVelocity);
            bRight.setVelocity(rightVelocity);
            fRight.setVelocity(rightVelocity);

            elbowPower = Range.clip(gamepad2.left_stick_y , -1.0, 1.0) * armSpeed;

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

            telemetry.addData("Claw"                , "Position: (%.2f)"         , claw.getPosition()
            );

            telemetry.addData("","----------");

            telemetry.addData("autoControl"         , "Start/Wobble: (%b), Top/Mid: (%b), Bot/Pickup: (%b), autoControl: (%b), Reached: (%b)", wobbleLevel, topMidLevel, autoControl, reached);


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