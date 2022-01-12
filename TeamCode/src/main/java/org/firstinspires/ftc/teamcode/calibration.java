package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="arm calibration", group="--")
//@Disabled
public class calibration extends LinearOpMode {

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

        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode((DcMotor.RunMode.RUN_USING_ENCODER));
        shoulder.setMode((DcMotor.RunMode.RUN_USING_ENCODER));
        // NEED TO PUT BOTH ARMS POINTING STRAIGHT UP INTO THE AIR (PERPENDICULAR TO THE CHASSIS/GROUND) TO RESET CORRECTLY FOR PRESET POSITIONS

        elbow.setDirection(DcMotorSimple.Direction.REVERSE);
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
            if (gamepad2.dpad_up) { // TOP    > shoulder = -50 , elbow = -185
                topLevel = true;
                midLevel = false;
                botLevel = false;
                pickupLevel = false;
                autoControl = true;
            }
            else if (gamepad2.dpad_right) { // MID    > shoulder = -150, elbow = -7
                topLevel = false;
                midLevel = true;
                botLevel = false;
                pickupLevel = false;
                autoControl = true;
            }
            else if (gamepad2.dpad_down) { // BOT    > shoulder =  145, elbow = -215
                topLevel = false;
                midLevel = false;
                botLevel = true;
                pickupLevel = false;
                autoControl = true;
            }
            else if (gamepad2.dpad_left) { // PICKUP > shoulder = 250, elbow = -330
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
            if (topLevel) {
                shoulder.setTargetPosition(0);
                elbow.setTargetPosition(170);
            }
            else if (midLevel) {
                shoulder.setTargetPosition(16);
                elbow.setTargetPosition(170);
            }
            else if (wobbleLevel) {
                shoulder.setTargetPosition(0);
                elbow.setTargetPosition(250);
            }
            else if (botLevel) {
                shoulder.setTargetPosition(248);
                elbow.setTargetPosition(5);
            }
            else if (pickupLevel) {
                shoulder.setTargetPosition(270);
                elbow.setTargetPosition(5);
                sleep(500);
                if (gamepad2.y){
                    claw.setPosition(0);
                }
            }

            if (autoControl) {
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shoulder.setVelocity(500);
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