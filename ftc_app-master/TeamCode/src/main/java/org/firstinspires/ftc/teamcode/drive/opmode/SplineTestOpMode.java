package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.AssetsTrajectoryLoader;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.tensorflow_vision.mineral_sampler;
import java.io.IOException;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous
public class SplineTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException
    {
        mineral_sampler vision = new mineral_sampler();
        vision.begin(hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        AssetsTrajectoryLoader loader = new AssetsTrajectoryLoader();
        Trajectory trajectory = null;
        drive.setPoseEstimate(new Pose2d(0, 0, 0));
        try {
            trajectory = AssetsTrajectoryLoader.load("test");
        } catch (IOException e) {
            e.printStackTrace();
        }
                /*drive.trajectoryBuilder()
                .splineTo(new Pose2d(40,40,0))
                .build();*/

        waitForStart();
        vision.update();
        telemetry.addData("chosen:",vision.chosen);
        telemetry.update();
        vision.end();

        if (isStopRequested()) return;
            drive.followTrajectory(trajectory);
            while (!isStopRequested() && drive.isFollowingTrajectory())
            {
                Pose2d currentPose = drive.getPoseEstimate();

                TelemetryPacket packet = new TelemetryPacket();
                Canvas fieldOverlay = packet.fieldOverlay();

                packet.put("x", currentPose.getX());
                packet.put("y", currentPose.getY());
                packet.put("heading", currentPose.getHeading());

                fieldOverlay.setStrokeWidth(4);
                fieldOverlay.setStroke("green");
                DashboardUtil.drawSampledTrajectory(fieldOverlay, trajectory);

                fieldOverlay.setFill("blue");
                fieldOverlay.fillCircle(currentPose.getX(), currentPose.getY(), 3);

                dashboard.sendTelemetryPacket(packet);

                drive.update();
            }
    }
}
