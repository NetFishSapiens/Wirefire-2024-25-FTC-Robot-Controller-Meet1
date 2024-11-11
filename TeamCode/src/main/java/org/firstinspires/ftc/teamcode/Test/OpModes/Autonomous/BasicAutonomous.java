package org.firstinspires.ftc.teamcode.Test.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Test.Structure.MainStructure;

@Autonomous(name = "BasicAutonomous", group = "Test")
public class BasicAutonomous extends LinearOpMode {

    MainStructure ms = new MainStructure();//ms = mainStructure

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        while(opModeIsActive()) {
        }
    }
    public void moveForward(double power, long time) {
        //set motor power
        ms.leftFront.setPower(power);
        ms.leftBack.setPower(power);
        ms.rightFront.setPower(power);
        ms.rightBack.setPower(power);

        //run the motor for set time
        sleep(time);
    }
    public void turnRight(double power, long time) {
        //set motor power
        ms.leftFront.setPower(-power);
        ms.leftBack.setPower(-power);
        ms.rightFront.setPower(power);
        ms.rightBack.setPower(power);

        //run the motor for set time
        sleep(time);
    }
    public void turnLeft(double power, long time) {
        //set motor power
        ms.leftFront.setPower(power);
        ms.leftBack.setPower(power);
        ms.rightFront.setPower(-power);
        ms.rightBack.setPower(-power);

        //run the motor for set time
        sleep(time);
    }
    public void stopMotor() {
        ms.leftFront.setPower(0);
        ms.leftBack.setPower(0);
        ms.rightFront.setPower(0);
        ms.rightBack.setPower(0);
    }
}
