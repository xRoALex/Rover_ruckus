package org.firstinspires.ftc.teamcode.driving;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.AssetsTrajectoryLoader;


@TeleOp(name="driving2", group="Iterative Opmode")
public class driving2  extends LinearOpMode
{

    public static final float  box_left_down = 0f,box_right_down=0f,box_left_up=0.87f,box_right_up=0.87f;
    public static final float box_blocker_blocked = 0.04f,box_blocker_release = 0.4f;

    float speed = 0.5f;

    robot2 rob = new robot2();
    SampleMecanumDriveBase drive ;



    public void runOpMode() {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        rob.init(hardwareMap);

        while(!opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("Waiting for start",getRuntime());
            telemetry.update();
        }

        while (opModeIsActive()) {


            if(gamepad2.left_bumper)
            {
                rob.linear_intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rob.linear_intake.setPower(0.7f);
            }
            else if(gamepad2.right_bumper)
            {
                rob.linear_intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rob.linear_intake.setPower(-0.7f);
            }
            else if(rob.linear_intake.getMode() == DcMotor.RunMode.RUN_USING_ENCODER)
            {
                rob.linear_intake.setPower(0f);
            }

            if(gamepad2.x)
            {
                rob.linear_intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rob.linear_intake.setTargetPosition(0);
                rob.linear_intake.setPower(0.7f);
                rob.box_blocker.setPosition(box_blocker_blocked);
                rob.box_right.setPosition(box_right_up);
                rob.box_left.setPosition(box_left_up);
                rob.intake.setPower(0f);
            }
            //box control

            if(gamepad2.dpad_up)
            {
               /* rob.box.setPower(0.8);
                rob.box.setTargetPosition(rob.box.getCurrentPosition()+50);*/
               rob.box_left.setPosition(box_left_up);
               rob.box_right.setPosition(box_right_up);
               /*rob.box_left.setPosition(rob.box_left.getPosition()+0.01f);
               rob.box_right.setPosition(rob.box_right.getPosition()+0.01f);*/
            }
            else
            if(gamepad2.dpad_down)
            {
                /*rob.box.setPower(0.8);
                rob.box.setTargetPosition(rob.box.getCurrentPosition()-50);*/
                rob.box_left.setPosition(box_left_down);
                rob.box_right.setPosition(box_right_down);
                /*rob.box_left.setPosition(rob.box_left.getPosition()-0.01f);
                rob.box_right.setPosition(rob.box_right.getPosition()-0.01f);*/
            }
            // BoxBlocker

            if(gamepad2.dpad_right)
            {
                rob.box_blocker.setPosition(rob.box_blocker.getPosition()+0.01f);

            }
            else if(gamepad2.dpad_left)
            {
                rob.box_blocker.setPosition(rob.box_blocker.getPosition()-0.01f);

            }

            // intake control

            if (gamepad2.a)
            {
                rob.box_blocker.setPosition(box_blocker_blocked);
                rob.intake.setPower(-1f);
            }
            else if(gamepad2.y)
            {
                rob.box_blocker.setPosition(box_blocker_release);
                rob.intake.setPower(1f);
            }
            else if(gamepad2.b)
            {
                rob.intake.setPower(0f);
                rob.box_blocker.setPosition(box_blocker_blocked);
            }


            // scoring linear control

            if(gamepad1.right_bumper)
            {
             rob.linear_score.setPower(0.7f);
            }
            else
            if(gamepad1.left_bumper)
            {
                rob.linear_score.setPower(-0.7f);
            }
            else
            {
                rob.linear_score.setPower(0);
            }

            if(gamepad1.dpad_left)
            {
                rob.servo_score.setPosition(rob.servo_score.getPosition()+0.02f);
            }
            else
            if(gamepad1.dpad_right)
            {
                rob.servo_score.setPosition(rob.servo_score.getPosition()-0.02f);
            }

            //drive base control

            if(gamepad1.dpad_down    5)
            {
                rob.box_blocker.setPosition(box_blocker_release);
                rob.intake.setPower(-1f);
            }
            float x = gamepad1.left_stick_x, y = gamepad1.left_stick_y, z = gamepad1.right_trigger - gamepad1.left_trigger;

            if(gamepad1.a)
                speed = 0.5f;
            if(gamepad1.start)
                speed = 1f;

                drive.setMotorPowers((-y + x + z)*speed, (-y + x - z)*speed, (-y - x + z)*speed, (-y - x - z)*speed);


            telemetry.addData("linear_intake:",rob.linear_intake.getCurrentPosition());
            telemetry.addData("servo_left:",rob.box_left.getPosition());
            telemetry.addData("servo_right:",rob.box_right.getPosition());
            telemetry.addData("blocker:",rob.box_blocker.getPosition());

            telemetry.update();


        }
    }
}
