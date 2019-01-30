package org.firstinspires.ftc.teamcode.driving;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class robot2 {
    public DcMotor linear_intake;
    public Servo box_left,box_right;
    public DcMotor intake;
    public Servo box_blocker;
    public Servo servo_score;
    public DcMotor linear_score;

    public robot2(){

    }


    public void init(HardwareMap ahwMap)
    {
        linear_intake = ahwMap.get(DcMotor.class,"linear_intake");
        linear_intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear_intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linear_intake.setDirection(DcMotor.Direction.FORWARD);
        linear_intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linear_score = ahwMap.get(DcMotor.class,"linear_score");
        linear_score.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linear_score.setDirection(DcMotor.Direction.FORWARD);
        linear_score.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       /* box = ahwMap.get(DcMotor.class,"box");
        box.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        box.setDirection(DcMotor.Direction.FORWARD);
        box.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/
       box_left = ahwMap.get(Servo.class,"box_left");
       box_left.setDirection(Servo.Direction.FORWARD);
       box_left.setPosition(0);

       box_right = ahwMap.get(Servo.class,"box_right");
       box_right.setDirection(Servo.Direction.REVERSE);
       box_right.setPosition(0);

       box_blocker = ahwMap.get(Servo.class,"box_blocker");
       box_blocker.setDirection(Servo.Direction.FORWARD);

        servo_score = ahwMap.get(Servo.class,"servo_score");
        servo_score.setDirection(Servo.Direction.FORWARD);

        intake = ahwMap.get(DcMotor.class,"intake");
        intake.setDirection(CRServo.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setPower(0f);

    }
}

