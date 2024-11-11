package org.firstinspires.ftc.teamcode.Test.Structure;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class MainStructure {

    public HardwareMap hardwareMap;//Refers to hardwareMap for Import

    public DcMotor leftFront; //Creates DcMotor leftFront, find in BasicTeleOp
    public DcMotor leftBack; //Creates DcMotor leftBack, find in BasicTeleOp
    public DcMotor rightFront; //Creates DcMotor rightFront, find in BasicTeleOp
    public DcMotor rightBack; //Creates DcMotor rightBack, find in BasicTeleOp
    public CRServo intakeservo; //Creates Servo claw, find in BasicTeleOp
    public Servo armservo;
    public DcMotor rightSlide;
    public DcMotor leftSlide;

    public MainStructure(){
        leftFront = hardwareMap.get(DcMotor.class, "frontleft");
        leftBack = hardwareMap.get(DcMotor.class, "backleft");
        rightFront = hardwareMap.get(DcMotor.class, "frontright");
        rightBack = hardwareMap.get(DcMotor.class, "backright");
        intakeservo = hardwareMap.get(CRServo.class, "intakeservo");
        armservo = hardwareMap.get(Servo.class, "armservo");
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");



    }
}
