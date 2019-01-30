package org.firstinspires.ftc.teamcode.driving;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class robot {
    public DcMotor arm_left=null,arm_right=null,front_left=null,front_right=null,back_left=null,back_right=null;
    public DcMotor intake;
    public DcMotor linear;
    public Servo box;
    public Servo marker;

    public robot(){

    }


    public void init(HardwareMap ahwMap)
    {
        arm_right = ahwMap.get(DcMotor.class,"arm_right");
        arm_left = ahwMap.get(DcMotor.class,"arm_left");

        arm_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        arm_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        arm_right.setDirection(DcMotor.Direction.REVERSE);
        arm_left.setDirection(DcMotor.Direction.FORWARD);


        arm_left.setPower(0f);
        arm_right.setPower(0f);


        arm_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        intake = ahwMap.get(DcMotor.class,"intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        linear  = ahwMap.get(DcMotor.class,"linear");
        linear.setDirection(DcMotor.Direction.FORWARD);
        linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        box = ahwMap.get(Servo.class,"box");
        box.setDirection(Servo.Direction.FORWARD);

        marker= ahwMap.get(Servo.class,"marker");
        marker.setDirection(Servo.Direction.FORWARD);

    }
}

