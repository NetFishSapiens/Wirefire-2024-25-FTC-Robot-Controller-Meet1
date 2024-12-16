package org.firstinspires.ftc.teamcode.WireFireFTC.OpModes.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "WireFire-TeleOp3",group = "Linear OpMode")
public class WireFireTeleOp3 extends LinearOpMode {
    //Used for Telemetry
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime delay_time = new ElapsedTime();

    //Used as Variables for the ARM
    int rotation = 0;
    double INCREMENT = 20;

    //Used as Variables for the Slides
    int height = 0;
    double HEIGHT_INCREMENT = 20;
    final int MAX_HEIGHT = 2000;
    final int MIN_HEIGHT = 0;
    double PWR_MULTIPLIER = 0.75;

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

        //arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset encoder

        //ArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Setting all target positions to zero so it doesn't jitter
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

            //Used for the intake servo
            if(gamepad2.right_trigger > 0) {
                intakeservo.setPower(2.5);
            }
            else if(gamepad2.left_trigger > 0) {
                intakeservo.setPower(-2.5);
            }
            else {
                intakeservo.setPower(0);
            }

            //Setting modes and ZeroPower
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //Increases/Decreases PWR_MULTIPLIER on up/down
            if(gamepad1.dpad_up) {
                    PWR_MULTIPLIER += 0.01;
            }
            else if(gamepad1.dpad_down){
                    PWR_MULTIPLIER -= 0.01;
            }

            //Used for Arm_Motor see slide code for details
            if(gamepad2.left_stick_y > 0.05) {
                rotation += INCREMENT;
                arm(rotation);
            } else if (gamepad2.left_stick_y < -0.05) {
                rotation -= INCREMENT;
                arm(rotation);
            }

            //Code for Slides using values to determine how long for the motors to be set until it reaches Target Position
            if(gamepad2.y) {
                height += HEIGHT_INCREMENT;
                Slide(height);
            } else if (gamepad2.a) {
                height -= HEIGHT_INCREMENT;
                Slide(height);
            }

            if(gamepad2.dpad_up){
                arm(-1900);
                Slide(2000);
                movement( 0.1, -0.5, 0.0, 0.0);
                stopMotors();
                intake_servo(2.5,1.0, -0.09);
            } else if (gamepad2.dpad_down) {
                arm(-3700);
                Slide(0);
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

            //Clamp PWR_Multiplier
            if(PWR_MULTIPLIER>1.0){
                PWR_MULTIPLIER = 1.0;
            }
            else if(PWR_MULTIPLIER < -1.0){
                PWR_MULTIPLIER = 0.25;
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
            telemetry.addData("PWR_Multiplier", PWR_MULTIPLIER);
            telemetry.update();
        }

        // Stop all motors when op mode is stopped
        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);
    }
    private void movement(double seconds, double Y_LEFT_INPUT, double X_LEFT_INPUT, double X_RIGHT_INPUT){
        // Get gamepad inputs
        double forward = -Y_LEFT_INPUT; // Forward/backward movement
        double strafe = X_LEFT_INPUT;  // Left/right movement
        double turn = X_RIGHT_INPUT;   // Turn left/right

        // Calculate motor powers
        double frontLeftPower = forward + strafe + turn;
        double frontRightPower = forward - strafe - turn;
        double backLeftPower = forward - strafe + turn;
        double backRightPower = forward + strafe - turn;

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

        sleep((long) (seconds * 1000));
    }
    // Method to stop the motors
    private void stopMotors() {
        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);
    }

    private void intake_servo(double delay, double seconds, double G2_Y_LEFT_INPUT) {
        delay_time.reset();
        if(delay_time.time() > delay) {
            intakeservo.setPower(G2_Y_LEFT_INPUT);
        }
        sleep((long) (seconds * 1000));
    }

    private void Slide(int Height){
        Height = Math.max(MIN_HEIGHT, Math.min(MAX_HEIGHT, height));
        leftSlide.setTargetPosition(Height);
        leftSlide.setPower(1);
        rightSlide.setTargetPosition(Height);
        rightSlide.setPower(1);
    }

    private void arm(int rot){
        arm_motor.setTargetPosition(rot);
        arm_motor.setPower(1);
    }
}