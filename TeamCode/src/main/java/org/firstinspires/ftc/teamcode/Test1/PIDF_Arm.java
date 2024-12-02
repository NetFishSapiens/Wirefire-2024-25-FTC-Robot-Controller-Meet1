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
public class PIDF_Arm extends OpMode {
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0.0;

    public static int target = 0;

    private final double ticks_in_degree = 537.6/360.0;

    private DcMotorEx arm_motor;
    @Override
    public void init() {
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor = hardwareMap.get(DcMotorEx.class, "arm_motor");
        //arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        double pwr = gamepad2.left_stick_y;
        double rsp = gamepad2.right_stick_y;
        controller.setPID(p,i,d);
        int armPos = arm_motor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;

        double power = pid + ff;

        if(power>1.0){
            power = 1.0;
        } else if(power < -1.0){
            power = -1.0;
        }
        if(pwr == 0){
            arm_motor.setPower(power*0.02);
        }
        else{
            arm_motor.setPower(pwr*0.1);
        }
        if(rsp > 0.5){
            target += 1;
        }
        else if(rsp < -0.5){
            target -= 1;
        }
        if(target < 0){
            target = 0;
        }else if(target > 200){
            target = 200;
        }
        telemetry.addData("Arm Position", armPos);
        telemetry.addData("Target Position", target);
        telemetry.addData("PID Output", pid);
        telemetry.addData("Feedforward", ff);
        telemetry.update();
    }
}
