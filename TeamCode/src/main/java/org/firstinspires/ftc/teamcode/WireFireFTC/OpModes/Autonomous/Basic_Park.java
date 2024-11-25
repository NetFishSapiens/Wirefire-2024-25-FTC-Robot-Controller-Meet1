package org.firstinspires.ftc.teamcode.WireFireFTC.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//Made similar to TeleOp d
@Autonomous(name = "Park")
public class Basic_Park extends LinearOpMode {

    private DcMotor frontleft;
    private DcMotor frontright;
    private DcMotor backleft;
    private DcMotor backright;

    @Override
    public void runOpMode() {
        initializeMotors();
        waitForStart();

        movement( 1.1, 0.0, 1.0, 0.0);
        stopMotors();
        movement( 0.5, 0.5, 0.0, 0.0);
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
}
