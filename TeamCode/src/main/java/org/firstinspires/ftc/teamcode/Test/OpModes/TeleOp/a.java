package org.firstinspires.ftc.teamcode.Test.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Basic_Movement")
public class a extends LinearOpMode {

    private DcMotor frontleft;
    private DcMotor frontright;
    private DcMotor backleft;
    private DcMotor backright;

    private CRServo intakeservo =null;
    private Servo armservo = null;
    double rotation = 0;
    double INCREMENT = 0.002;

    @Override
    public void runOpMode() {
        // Initialize the Motors
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");

        //Initialize the Servos
        armservo = hardwareMap.get(Servo.class, "armservo");
        //Initialize the CR_Servo
        intakeservo = hardwareMap.get(CRServo.class, "intakeservo");

        // Set motor directions if needed
        frontleft.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.FORWARD);
        backright.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Main control loop
        while (opModeIsActive()) {
            //Adding telemetry for rotation
            telemetry.addData("rotation", rotation);
            telemetry.update();

            // Get gamepad inputs
            double forward = -gamepad1.left_stick_y; // Forward/backward movement
            double strafe = gamepad1.left_stick_x;  // Left/right movement
            double turn = gamepad1.right_stick_x;   // Turn left/right

            // Calculate motor powers
            double frontLeftPower = forward + strafe + turn;
            double frontRightPower = forward - strafe - turn;
            double backLeftPower = forward - strafe + turn;
            double backRightPower = forward + strafe - turn;

            if(gamepad2.right_trigger > 0) {
                intakeservo.setPower(1.5);
            }
            else if(gamepad2.left_trigger > 0) {
                intakeservo.setPower(-1.5);
            }
            else {
                intakeservo.setPower(0);
            }
            if(Math.abs(gamepad2.left_stick_y) > 0){
                rotation += INCREMENT * -gamepad2.left_stick_y;
                rotation = Math.max(0, Math.min(1, rotation));
                armservo.setPosition(rotation);
            }

            // Normalize motor powers to stay within the range of -1 to 1
            double maxPower = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
            if (maxPower > 1) {
                frontLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backLeftPower /= maxPower;
                backRightPower /= maxPower;
            }

            // Set motor powers
            frontleft.setPower(frontLeftPower);
            frontright.setPower(frontRightPower);
            backleft.setPower(backLeftPower);
            backright.setPower(backRightPower);

            // Optional telemetry
            telemetry.addData("Front Left Power", frontLeftPower);
            telemetry.addData("Front Right Power", frontRightPower);
            telemetry.addData("Back Left Power", backLeftPower);
            telemetry.addData("Back Right Power", backRightPower);
            telemetry.update();
        }

        // Stop all motors when op mode is stopped
        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);
    }
}