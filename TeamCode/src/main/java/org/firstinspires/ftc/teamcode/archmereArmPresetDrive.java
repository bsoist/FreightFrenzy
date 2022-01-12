package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@TeleOp(name="ArchmereDrive", group="--")
//@Disabled
public class archmereArmPresetDrive extends LinearOpMode {

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

        bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bRight.setDirection(DcMotor.Direction.REVERSE);
        fRight.setDirection(DcMotor.Direction.REVERSE);

        double armSpeed = .75;
        double elbowPower = 0;
        double shoulderPower = 0;
        claw.setPosition(-1.0);
        boolean elbowBrake = false;
        boolean shoulderBrake = false;
        boolean turnTable = false;
        boolean topLevel = false;
        boolean midLevel = false;
        boolean botLevel = false;
        boolean wobbleLevel = false;
        boolean pickupLevel = false;
        boolean autoControl = false;



        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //claw controls
            if (gamepad2.a) {
                claw.setPosition(1.0);
            }
            else if (gamepad2.y) {
                if (autoControl) {
                    if (midLevel || botLevel) {
                        claw.setPosition(.8);
                    }
                    else {
                        claw.setPosition(.4);
                    }
                }
                else {
                    claw.setPosition(.2);
                }
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

            if (gamepad2.dpad_up) { // TOP
                topLevel = true;
                midLevel = false;
                botLevel = false;
                pickupLevel = false;
                autoControl = true;
            }
            else if (gamepad2.dpad_right) { // MID
                topLevel = false;
                midLevel = true;
                botLevel = false;
                pickupLevel = false;
                autoControl = true;
            }
            else if (gamepad2.dpad_down) { // BOT/WOBBLE
                topLevel = false;
                midLevel = false;
                botLevel = true;
                pickupLevel = false;
                autoControl = true;
            }
            else if (gamepad2.dpad_left) { // PICKUP
                topLevel = false;
                midLevel = false;
                botLevel = false;
                pickupLevel = true;
                autoControl = true;
            }
            else if (gamepad2.left_bumper) { //AUTO CONTROL OFF
                topLevel = false;
                midLevel = false;
                botLevel = false;
                pickupLevel = false;
                autoControl = false;
            }
            if (topLevel) { //good to go for ths meet
                shoulder.setTargetPosition(0);
                elbow.setTargetPosition(-160);
            }
            else if (midLevel) { //good to go for ths meet
                shoulder.setTargetPosition(0);
                elbow.setTargetPosition(-240);
            }
            else if (wobbleLevel) {
                shoulder.setTargetPosition(0);
                elbow.setTargetPosition(250);
            }
            else if (botLevel) {
                shoulder.setTargetPosition(248);
                elbow.setTargetPosition(5);
            }
            else if (pickupLevel) { //good to go for ths meet
                shoulder.setTargetPosition(-2057);
                elbow.setTargetPosition(-135);
            }
            if (autoControl) {
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shoulder.setVelocity(1200);
                elbow.setVelocity(500);

            }
            else {
                shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                shoulder.setPower(shoulderPower);
                elbow.setPower(elbowPower);
            }

            int elbowCurrentPosition    = elbow   .getCurrentPosition();
            int shoulderCurrentPosition = shoulder.getCurrentPosition();
//            int bLeftCP                 = bLeft   .getCurrentPosition();
//            int bRightCP                = bRight  .getCurrentPosition();
//            int fLeftCP                 = fLeft   .getCurrentPosition();
//            int fRightCP                = fRight  .getCurrentPosition();

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

            elbowPower    = Range.clip(gamepad2.left_stick_y , -1.0, 1.0) * armSpeed;
            shoulderPower = Range.clip(gamepad2.right_stick_y, -1.0, 1.0) * armSpeed;

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


            if (!autoControl) {
                if (elbowBrake) { // brakes only work if using arm manually
                    elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elbow.setVelocity(1000);
                }
                else {
                    elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    elbow.setPower(elbowPower);
                }

                if (shoulderBrake) {
                    shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    shoulder.setVelocity(400);
                }
                else {
                    shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    shoulder.setPower(shoulderPower);
                }
            }
//            if (autoControl) {
//                if (shoulderPower != 0) {
//                    int newShoulderPosition = shoulder.getTargetPosition();
//                    if (shoulderPower < 0) { //shoulder power is reversed, up = negatve
//                        newShoulderPosition --; //clockwise, left (or upward) rotation = negative
//                        shoulder.setTargetPosition(newShoulderPosition);
//                    }
//                    else if (shoulderPower > 0) { //down = positive
//                        newShoulderPosition ++; //right (or downward) rotation = posiive
//                        shoulder.setTargetPosition(newShoulderPosition);
//                    }
//                }
//                if (elbowPower != 0) {
//                    int newElbowPosition = elbow.getTargetPosition();
//                    if (elbowPower < 0) { //shoulder power is reversed, up = negatve
//                        newElbowPosition --; //clockwise, left (or upward) rotation = negative
//                        elbow.setTargetPosition(newElbowPosition);
//                    }
//                    else if (elbowPower > 0) { //down = positive
//                        newElbowPosition ++; //right (or downward) rotation = posiive
//                        elbow.setTargetPosition(newElbowPosition);
//                    }
//                }
//            }



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status"       , "Run Time: " + runtime.toString());
            telemetry.addData("Base Velocity", "(%.2f)"    , motorVelocity     );

            telemetry.addData("Motor Power",
                    "back left (%.2f), back right (%.2f), front left (%.2f), front right (%.2f)",
                    bLeft.getPower(), bRight.getPower(), fLeft.getPower(),  fRight.getPower());

            telemetry.addData("Theoretical Velocity", "left (%.2f), right (%.2f)", leftVelocity , rightVelocity );
            telemetry.addData("Motor Direction"     , "left (%.2f), right (%.2f)", leftDirection, rightDirection);
            telemetry.addData("Claw"                , "Position: (%.2f)"         , claw.getPosition()           );

            telemetry.addData("Shoulder",
                    "Power: (%.2f), Position: (%d), Target Position: (%d), Brake: (%b)",
                    shoulderPower, shoulderCurrentPosition, shoulder.getTargetPosition(), shoulderBrake);
            telemetry.addData("Elbow",
                    "Power: (%.2f), Position: (%d), Target Position: (%d), Brake: (%b)",
                    elbowPower   , elbowCurrentPosition   , elbow.getTargetPosition()    , elbowBrake  );

            telemetry.addData("autoControl"         , "top: (%b), mid: (%b), bot: (%b), pickup: (%b), autoControl: (%b)", topLevel, midLevel, botLevel, pickupLevel, autoControl);
            telemetry.update();
        }
    }
}