package org.firstinspires.ftc.teamcode.driving;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.AssetsTrajectoryLoader;


@TeleOp(name="driving", group="Iterative Opmode")
public class driving  extends LinearOpMode
{
    double dropxPos = 19.8, dropyPos = -35.3,dropheadPos = 0.73;
    double pickxPos,pickyPos,pickheadPos;
    double boxPos;
    float speed = 0.5f;
    int drop_a_left = -835,drop_a_right = -874;
    int pick_a_left,pick_a_right;
    AssetsTrajectoryLoader loader = new AssetsTrajectoryLoader();
    Pose2d curent ;
    robot rob = new robot();
    Trajectory trajectory;
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


            // linear slide control

            if(gamepad2.right_bumper)
            {
                rob.linear.setPower(0.8f);
            }
            else if(gamepad2.left_bumper)
            {
                rob.linear.setPower(-0.8f);
            }
            else
            {
                rob.linear.setPower(0);
            }


            // box control

            if(gamepad2.dpad_right)
            {
                rob.box.setPosition(Math.max(0.34,rob.box.getPosition()-0.02f));
            }
            else
             if(gamepad2.dpad_left)
            {
                rob.box.setPosition(rob.box.getPosition()+0.02f);
            }

            // intake control

            if (gamepad2.y)
            {
                rob.intake.setPower(-0.3f);
            }
            else if(gamepad2.a)
            {
                rob.intake.setPower(0.8f);
            }
            else if(gamepad2.b)
                rob.intake.setPower(0f);


            // arm control up/down
            if (gamepad1.left_bumper)
            {
                rob.arm_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rob.arm_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rob.arm_right.setPower(0.5f);
                rob.arm_left.setPower(0.5f);
            }
            else if (gamepad1.right_bumper)
            {
                rob.arm_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rob.arm_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rob.arm_right.setPower(-0.5f);
                rob.arm_left.setPower(-0.5f);

            }
            else /*if(rob.arm_left.getMode() == DcMotor.RunMode.RUN_USING_ENCODER)
                {
                    rob.arm_right.setPower(0);
                    rob.arm_left.setPower(0);
            }*/
                if(rob.arm_left.getMode() == DcMotor.RunMode.RUN_USING_ENCODER && !drive.isFollowingTrajectory() )
                {
                   /* rob.arm_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rob.arm_left.setMod0000e(DcMotor.RunMode.RUN_TO_POSITION);*/
                    rob.arm_right.setPower(0f);
                    rob.arm_left.setPower(0f);
                   /* rob.arm_left.setTargetPosition(rob.arm_left.getCurrentPosition());
                    rob.arm_right.setTargetPosition(rob.arm_right.getCurrentPosition());*/
                }

            // set arm power for latching

            if(gamepad1.y)
            {
                rob.arm_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rob.arm_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rob.arm_right.setPower(1f);
                rob.arm_left.setPower(1f);
                rob.arm_left.setTargetPosition(rob.arm_left.getCurrentPosition() + 200);
                rob.arm_right.setTargetPosition(rob.arm_right.getCurrentPosition() + 200);
            }
            else if(gamepad1.x)
            {
                rob.arm_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rob.arm_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rob.arm_right.setPower(1f);
                rob.arm_left.setPower(1f);
                rob.arm_left.setTargetPosition(rob.arm_left.getCurrentPosition() - 200);
                rob.arm_right.setTargetPosition(rob.arm_right.getCurrentPosition() - 200);
            }

            // set motor powers if not following a trajectory

            float x = gamepad1.left_stick_x, y = gamepad1.left_stick_y, z = gamepad1.right_trigger - gamepad1.left_trigger;

            if(gamepad1.a)
                speed = 0.5f;
            if(gamepad1.start)
                speed = 1f;
            if(!drive.isFollowingTrajectory())
            {
                    drive.setMotorPowers((-y + x + z)*speed, (-y + x - z)*speed, (-y - x + z)*speed, (-y - x - z)*speed);
            }


            if(gamepad1.dpad_down) // save scoring location
            {
                dropxPos=curent.getX();
                dropyPos=curent.getY();
                dropheadPos=curent.getHeading();
                drop_a_right = rob.arm_right.getCurrentPosition();
                drop_a_left = rob.arm_left.getCurrentPosition();
            }

            // updating position and follower used by the drive class

            curent = drive.getPoseEstimate();

            if(drive.isFollowingTrajectory())
            {
                drive.update();
            }
            else
                drive.updatePoseEstimate();

            // setting starting position after autonomy

            if(gamepad1.dpad_right)
            {
                drive.setPoseEstimate(new Pose2d(0,0,0));
            }

            // autonomous moving in teleop

            if (gamepad1.dpad_up && !drive.isFollowingTrajectory()) // going to score at the lander
            {
                    // save return location
                    boxPos = rob.box.getPosition();
                    pickxPos=curent.getX();
                    pickyPos=curent.getY();
                    pickheadPos=curent.getHeading();
                    pick_a_right = rob.arm_right.getCurrentPosition() - 500;
                    pick_a_left = rob.arm_left.getCurrentPosition() - 500;

                    // setup trajectory and arm position

                    rob.arm_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rob.arm_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rob.arm_right.setTargetPosition(drop_a_right);
                    rob.arm_left.setTargetPosition(drop_a_left);
                    rob.arm_right.setPower(0.4f);
                    rob.arm_left.setPower(0.4f);

                    drive.update();
                    trajectory = drive.trajectoryBuilder()
                            .setReversed(true)
                            .splineTo(new Pose2d(dropxPos, dropyPos, dropheadPos))
                            .build();

                    drive.followTrajectory(trajectory);
                    rob.box.setPosition(0.34f);
                    rob.intake.setPower(0);
            }

            if(gamepad1.dpad_left && !drive.isFollowingTrajectory())//  going back to collecting
                {
                    rob.box.setPosition(boxPos);

                    rob.arm_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rob.arm_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rob.arm_right.setTargetPosition(pick_a_right);
                    rob.arm_left.setTargetPosition(pick_a_left);
                    rob.arm_right.setPower(0.4f);
                    rob.arm_left.setPower(0.4f);

                    drive.update();
                    trajectory = drive.trajectoryBuilder()
                            .splineTo(new Pose2d(pickxPos, pickyPos, pickheadPos))
                            .build();
                    drive.followTrajectory(trajectory);
                    rob.intake.setPower(0.8f);
                }
//

            // Emergency stop
            if(gamepad1.b && drive.isFollowingTrajectory())
            {
                rob.arm_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rob.arm_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rob.arm_right.setPower(0);
                rob.arm_left.setPower(0);

                Trajectory traj = new Trajectory();
                drive.followTrajectory(traj);
            }



            /*telemetry.addData("box: ", rob.box.getPosition());
            telemetry.addData("x: ", curent.getX());
            telemetry.addData("y: ", curent.getY());
            telemetry.addData("heading: ", curent.getHeading());
            telemetry.addData("arm left : ", rob.arm_left.getCurrentPosition());
            telemetry.addData("arm right: ", rob.arm_right.getCurrentPosition());
            telemetry.addData("Brat: ", rob.linear.getCurrentPosition());*/
            telemetry.addData("vertical:",drive.getVerticalHeading());
            telemetry.update();


        }
    }
}
