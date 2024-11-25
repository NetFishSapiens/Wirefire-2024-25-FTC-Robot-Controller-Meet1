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
@TeleOp(name="Arm_Test3", group="Linear OpMode")
public class PIDF_ARMAltered extends OpMode {
    private PIDController controller;

    // PIDF constants, which can be adjusted via the dashboard
    public static double p = 0, i = 0, d = 0;
    public static double f = 0.1;

    public static int target = 59; // The target position in degrees

    private final double ticks_in_degree = 537.7 / 360.0; // Conversion factor for ticks to degrees

    private DcMotorEx arm_motor;

    @Override
    public void init() {
        // Initialize the PID controller with the provided constants
        controller = new PIDController(p, i, d);

        // Set up telemetry to send data to the FtcDashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize the arm motor and set it to use encoders
        arm_motor = hardwareMap.get(DcMotorEx.class, "arm_motor");
        arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset encoder
        arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Use encoder for feedback
    }

    @Override
    public void loop() {
        // Update the PID controller with the current p, i, and d values
        controller.setPID(p, i, d);

        // Get the current position of the arm (in encoder ticks)
        int armPos = arm_motor.getCurrentPosition();

        // Calculate the PID output based on the arm position and target
        double pid = controller.calculate(armPos, target);

        // Convert encoder ticks to degrees for feedforward calculation
        double currentDegrees = armPos * ticks_in_degree;

        // Calculate the feedforward term using the cosine of the current arm position in degrees
        double ff = Math.cos(Math.toRadians(currentDegrees)) * f;

        // Combine the PID output and feedforward term
        double power = pid + ff;

        // Clamp the motor power between -1.0 and 1.0 to avoid exceeding motor limits
        if (power > 1.0) {
            power = 1.0;
        } else if (power < -1.0) {
            power = -1.0;
        }

        // Set the motor power
        arm_motor.setPower(power);

        // Send telemetry data for debugging
        telemetry.addData("Arm Position (ticks)", armPos);
        telemetry.addData("Target Position (degrees)", target);
        telemetry.addData("PID Output", pid);
        telemetry.addData("Feedforward", ff);
        telemetry.addData("Motor Power", power);
        telemetry.update();
    }
}
