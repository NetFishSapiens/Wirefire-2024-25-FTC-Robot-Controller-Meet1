package org.firstinspires.ftc.teamcode.WireFireFTC.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//Made similar to TeleOp d
@Autonomous(name = "Touch_Bar")
public class Touch_Bar extends LinearOpMode {

    private ElapsedTime delay_time = new ElapsedTime();

    private DcMotor frontleft;
    private DcMotor frontright;
    private DcMotor backleft;
    private DcMotor backright;

    private DcMotor leftSlide = null;
    private DcMotor rightSlide = null;

    private DcMotorEx arm_motor;

    //Create the objects for servos
    private CRServo intakeservo = null;

    //Used as Variables for the Slides
    int height = 0;
    double HEIGHT_INCREMENT = 20;
    final int MAX_HEIGHT = 2000;
    final int MIN_HEIGHT = 0;
    double PWR_MULTIPLIER = 0.75;

    @Override
    public void runOpMode() {
        initializeMotors();
        initializeServos();
        waitForStart();

        movement( 0.8, -1.0, 0.0, 0.0);
        stopMotors();
        movement( 0.5, 0.0, 0.0, 0.9);
        stopMotors();
        movement( 0.5, -0.4, 0.0, 0.0);
        stopMotors();
        arm(-2000);
        stopMotors();
    }

    // Method to initialize the motors
    private void initializeMotors() {
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");

        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");

        arm_motor = hardwareMap.get(DcMotorEx.class, "arm_motor");

        // Set motor directions if needed
        frontleft.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.FORWARD);
        backright.setDirection(DcMotor.Direction.FORWARD);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);
    }

    private void initializeServos() {
        //Initialize the Servos
        //armservo = hardwareMap.get(Servo.class, "armservo");
        //Initialize the CR_Servo
        intakeservo = hardwareMap.get(CRServo.class, "intakeservo");
    }

    // Method to stop the motors
    private void stopMotors() {
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

    private void intake(double delay, double seconds, double G2_Y_LEFT_INPUT) {
        delay_time.reset();
        if(delay_time.time() > delay) {
            intakeservo.setPower(G2_Y_LEFT_INPUT);
        }
        sleep((long) (seconds * 1000));
    }
}
