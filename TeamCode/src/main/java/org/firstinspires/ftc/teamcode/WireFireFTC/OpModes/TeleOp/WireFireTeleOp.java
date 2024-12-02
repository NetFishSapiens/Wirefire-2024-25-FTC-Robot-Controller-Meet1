package org.firstinspires.ftc.teamcode.WireFireFTC.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
@TeleOp(name = "WireFireFTC TeleOp",group = "Linear OpMode")
public class WireFireTeleOp extends LinearOpMode {
    //Used for Telemetry
    private ElapsedTime runtime = new ElapsedTime();

    //Used as Variables for the ARM
    int rotation = 0;
    double INCREMENT = 2;
    final int MAX_ROTATION = 2200;
    final int MIN_ROTATION = 0;

    //Used as Variables for the Slides
    int height = 0;
    double HEIGHT_INCREMENT = 20;
    final int MAX_HEIGHT = 2000;
    final int MIN_HEIGHT = 0;
    double PWR_MULTIPLIER = 0.73;

    //Create the objects for motors
    private DcMotor frontleft = null;
    private DcMotor frontright = null;
    private DcMotor backleft = null;
    private DcMotor backright = null;
    private DcMotor leftSlide = null;
    private DcMotor rightSlide = null;
    private DcMotorEx arm_motor = null;

    //Create the objects for servos
    private CRServo intakeservo = null;
    //private Servo armservo = null;

    @Override
    public void runOpMode() {
        // Initialize the Motors
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        arm_motor = hardwareMap.get(DcMotorEx.class, "arm_motor");

        //Initialize the Servos
        //armservo = hardwareMap.get(Servo.class, "armservo");
        //Initialize the CR_Servo
        intakeservo = hardwareMap.get(CRServo.class, "intakeservo");

        // Set motor directions if needed
        frontleft.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.FORWARD);
        backright.setDirection(DcMotor.Direction.FORWARD);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);

        arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset encoder

        //ArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        rightSlide.setTargetPosition(0);
        leftSlide.setTargetPosition(0);
        arm_motor.setTargetPosition(0);
        runtime.reset();

        // Main control loop
        while (opModeIsActive()) {
            //Adding telemetry for rotation
            telemetry.addData("rotation", rotation);
            telemetry.update();

            if(gamepad2.right_trigger > 0) {
                intakeservo.setPower(1.5);
            }
            else if(gamepad2.left_trigger > 0) {
                intakeservo.setPower(-1.5);
            }
            else {
                intakeservo.setPower(0);
            }
            /*if(Math.abs(gamepad2.left_stick_y) > 0){
                rotation += INCREMENT * -gamepad2.left_stick_y;
                rotation = Math.max(0, Math.min(1, rotation));
                ArmMotor.setTargetPosition(rotation);
            }*/

            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            if(gamepad2.left_stick_y > 0.0) {
                rotation += INCREMENT;
                //height = Math.max(MIN_HEIGHT, Math.min(MAX_HEIGHT, height));
                arm_motor.setTargetPosition(rotation);
                arm_motor.setPower(1);
            } else if (gamepad2.left_stick_y < 0) {
                rotation -= INCREMENT;
                arm_motor.setTargetPosition(rotation);
                arm_motor.setPower(1);
            } else{
                // arm_motor.setPower(0);
            }

            if(gamepad2.y) {
                height += HEIGHT_INCREMENT;
                height = Math.max(MIN_HEIGHT, Math.min(MAX_HEIGHT, height));
                leftSlide.setTargetPosition(height);
                leftSlide.setPower(1);
                rightSlide.setTargetPosition(height);
                rightSlide.setPower(1);
            } else if (gamepad2.a) {
                height -= HEIGHT_INCREMENT;
                height = Math.max(MIN_HEIGHT, Math.min(MAX_HEIGHT, height));
                leftSlide.setTargetPosition(height);
                leftSlide.setPower(1);
                rightSlide.setTargetPosition(height);
                rightSlide.setPower(1);
            }

            // Get gamepad inputs
            double forward = -gamepad1.left_stick_y; // Forward/backward movement
            double strafe = gamepad1.left_stick_x;  // Left/right movement
            double turn = gamepad1.right_stick_x;   // Turn left/right

            // Calculate motor powers
            double frontLeftPower = forward + strafe + turn;
            double frontRightPower = forward - strafe - turn;
            double backLeftPower = forward - strafe + turn;
            double backRightPower = forward + strafe - turn;

            // Normalize motor powers to stay within the range of -1 to 1
            double max;
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            // Set motor powers
            frontleft.setPower(frontLeftPower*PWR_MULTIPLIER);
            frontright.setPower(frontRightPower*PWR_MULTIPLIER);
            backleft.setPower(backLeftPower*PWR_MULTIPLIER);
            backright.setPower(backRightPower*PWR_MULTIPLIER);

            // Optional telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            telemetry.addData("height:", height);
            telemetry.addData("left slide ticks:", leftSlide.getCurrentPosition());
            telemetry.addData("right slide ticks:", rightSlide.getCurrentPosition());
            telemetry.addData("rotation:", rotation);
            telemetry.addData("Arm motor ticks:", arm_motor.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motors when op mode is stopped
        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);
    }
}