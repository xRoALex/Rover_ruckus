package org.firstinspires.ftc.teamcode.autonomies;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.tensorflow_vision.mineral_sampler;
import org.firstinspires.ftc.teamcode.util.AssetsTrajectoryLoader;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.driving.robot;

import java.io.IOException;

@Autonomous
public class scurt extends LinearOpMode
{
    private static final String paths[] =new String[3];
    robot rob = new robot();
    public void runOpMode()
    {


        //initilaizing paths
        paths[0]="scurt_stanga";
        paths[1]="scurt_mijloc";
        paths[2]="scurt_dreapta";

        // initializing tensorflow object detection
        mineral_sampler vision = new mineral_sampler();
        vision.begin(hardwareMap);
        telemetry.addData("starting...",getRuntime());
        telemetry.update();

        // initializing web dashboard and the mecanum drive
        FtcDashboard dashboard = FtcDashboard.getInstance();
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        // setting up the lowering and parking trajectories;
        AssetsTrajectoryLoader loader = new AssetsTrajectoryLoader();
        Trajectory trajectory[] = new Trajectory[5];

        telemetry.addData("latching...",getRuntime());
        telemetry.update();

        rob.init(hardwareMap);
        rob.arm_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rob.arm_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rob.arm_left.setTargetPosition(3300);
        rob.arm_right.setTargetPosition(3300);
        rob.arm_left.setPower(0.5f);
        rob.arm_right.setPower(0.5f);

        // de-latcking
        trajectory[0] = drive.trajectoryBuilder()
                .strafeLeft(5)
                .build();

        // parking on the crater
        try {
            trajectory[2] = loader.load("lung_parcare");
        }
        catch (IOException e) {
            e.printStackTrace();
        }

        while(!opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("Waiting for start",getRuntime());
            telemetry.update();
        }

        // getting the final yellow cube position and adding the proper trajectory
        vision.update();
        try {
            trajectory[1] = loader.load(paths[vision.chosen]);
        } catch (IOException e) {
            e.printStackTrace();
        }

        vision.end(); // closing tensorflow

        if (isStopRequested()) return;

        for(int i=0;i<3;i++) // iterating through the trajectories
        {
            // lower the arm before going awayfrom the hook
            if(i==0)
            {
                lower_arm();
            }
            if(i==1)
            {
                drive.setPoseEstimate(new Pose2d(20,20,45));
                lower_arm2();
            }
            drive.followTrajectory(trajectory[i]);
            while (!isStopRequested() && drive.isFollowingTrajectory())
            {
                telemetry.addData("Following trajectory....", getRuntime());
                telemetry.update();
                Pose2d currentPose = drive.getPoseEstimate();

                TelemetryPacket packet = new TelemetryPacket();
                Canvas fieldOverlay = packet.fieldOverlay();

                packet.put("x", currentPose.getX());
                packet.put("y", currentPose.getY());
                packet.put("heading", currentPose.getHeading());

                fieldOverlay.setStrokeWidth(4);
                fieldOverlay.setStroke("green");
                DashboardUtil.drawSampledTrajectory(fieldOverlay, trajectory[i]);

                fieldOverlay.setFill("blue");
                fieldOverlay.fillCircle(currentPose.getX(), currentPose.getY(), 3);

                dashboard.sendTelemetryPacket(packet);

                drive.update();
            }

            if(i==1) // if we are in the depot, drop the Team Marker
            {
                drop_marker();
            }
        }
    }

    void drop_marker()
    {
        rob.marker.setPosition(1f);
    }

    void lower_arm2()
    {
        rob.arm_right.setPower(0.5f);
        rob.arm_left.setPower(0.5f);
        rob.arm_right.setTargetPosition(1800);
        rob.arm_left.setTargetPosition(1800);

    }

    void lower_arm()
    {

        rob.arm_right.setPower(0.5f);
        rob.arm_left.setPower(0.5f);
        rob.arm_right.setTargetPosition(0);
        rob.arm_left.setTargetPosition(0);
        while(rob.arm_right.isBusy() || rob.arm_left.isBusy())
        {

        }
    }
}
