package org.firstinspires.ftc.teamcode.Test1;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Arm_Test1", group="Linear OpMode")
public class ArmControlTeleOp extends OpMode {

    private DcMotor armMotor;  // Arm motor reference

    @Override
    public void init() {
        // Initialize the arm motor from the hardware map
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        // Check the joystick value to determine arm movement
        double armPower = gamepad2.left_stick_y; // Get vertical input from the left joystick
        if(armPower>1.0){
            armPower = 1.0;
        } else if(armPower < -1.0){
            armPower = -1.0;
        }
        // If joystick is centered (no input), stop the motor
        if (armPower == 0) {
            armMotor.setPower(0); // Stop motor when no input
        } else{
            armMotor.setPower(armPower); // Move motor according to joystick input
        }

        // Telemetry to display motor status and power
        telemetry.addData("Arm Power", armPower);
        telemetry.addData("Motor Power", armMotor.getPower());
        telemetry.update();
    }
}
