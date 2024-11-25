package org.firstinspires.ftc.teamcode.Test.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//Made similar to TeleOp d
@Autonomous(name = "Touch_Bar")
public class b extends LinearOpMode {

    private DcMotor frontleft;
    private DcMotor frontright;
    private DcMotor backleft;
    private DcMotor backright;

    private CRServo intakeservo =null;
    private Servo armservo = null;
    double rotation = 0.55;
    double INCREMENT = 0.002;

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
        arm(5.0, -0.5);
        stopMotors();
    }

    // Method to initialize the motors
    private void initializeMotors() {
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");

        // Set motor directions if needed
        frontleft.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.FORWARD);
        backright.setDirection(DcMotor.Direction.FORWARD);
    }

    private void initializeServos() {
        //Initialize the Servos
        armservo = hardwareMap.get(Servo.class, "armservo");
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
    private void arm(double seconds, double G2_Y_LEFT_INPUT){
        if(Math.abs(G2_Y_LEFT_INPUT) > 0){
            rotation += INCREMENT * -G2_Y_LEFT_INPUT;
            rotation = Math.max(0, Math.min(1, rotation));
            armservo.setPosition(rotation);
        }
        sleep((long) (seconds * 1000));
    }
}