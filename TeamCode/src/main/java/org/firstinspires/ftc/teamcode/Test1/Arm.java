package org.firstinspires.ftc.teamcode.Test1;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name="Arm_Test2", group="Linear OpMode")
public class Arm extends OpMode {
    int height = 0;
    double HEIGHT_INCREMENT = 2;
    final int MAX_HEIGHT = 2000;
    final int MIN_HEIGHT = 0;
    double PWR_MULTIPLIER = 0.73;

    private DcMotorEx arm_motor;
    @Override
    public void init() {
        arm_motor = hardwareMap.get(DcMotorEx.class, "arm_motor");
        arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset encoder
        arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        arm_motor.setTargetPosition(0);
    }

    @Override
    public void loop() {
        arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if(gamepad2.left_stick_y > 0.0) {
            height += HEIGHT_INCREMENT;
            //height = Math.max(MIN_HEIGHT, Math.min(MAX_HEIGHT, height));
            arm_motor.setTargetPosition(height);
            arm_motor.setPower(1);
        } else if (gamepad2.left_stick_y < 0) {
            height -= HEIGHT_INCREMENT;
            arm_motor.setTargetPosition(height);
            arm_motor.setPower(1);
        } else{
           // arm_motor.setPower(0);
        }
        telemetry.addData("height:", height);
        telemetry.addData("Arm motor ticks:", arm_motor.getCurrentPosition());
        telemetry.update();
    }
}
