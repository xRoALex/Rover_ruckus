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
public class basic_scurt extends LinearOpMode
{
    robot rob = new robot();
    public void runOpMode()
    {


        //initilaizing paths
        // initializing tensorflow object detection

        // initializing web dashboard and the mecanum drive
        FtcDashboard dashboard = FtcDashboard.getInstance();
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        // setting up the lowering and parking trajectories;
        AssetsTrajectoryLoader loader = new AssetsTrajectoryLoader();
        Trajectory trajectory[] = new Trajectory[5];

        telemetry.addData("latching...",getRuntime());
        telemetry.update();

        rob.init(hardwareMap);


        // de-latcking
        /*trajectory[0] = drive.trajectoryBuilder()
                .strafeLeft(5)
                .build();*/

        // parking on the crater
        try {
            trajectory[1] = loader.load("scurt_parcare");
        }
        catch (IOException e) {
            e.printStackTrace();
        }
        try {
            trajectory[0] = loader.load("scurt_mijloc");
        } catch (IOException e) {
            e.printStackTrace();
        }
        while(!opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("Waiting for start",getRuntime());
            telemetry.update();
        }

        // getting the final yellow cube position and adding the proper trajectory


        if (isStopRequested()) return;

        for(int i=0;i<2;i++) // iterating through the trajectories
        {
            // lower the arm before going awayfrom the hook
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

            if(i==0) // if we are in the depot, drop the Team Marker
            {
                drop_marker();
            }
        }
    }

    void drop_marker()
    {
        rob.marker.setPosition(1f);
    }


}
