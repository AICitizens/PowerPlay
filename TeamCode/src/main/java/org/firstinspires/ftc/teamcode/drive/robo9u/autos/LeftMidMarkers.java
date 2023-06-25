package org.firstinspires.ftc.teamcode.drive.robo9u.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.robo9u.Modules.Detection;
import org.firstinspires.ftc.teamcode.drive.robo9u.Modules.Lift;
import org.firstinspires.ftc.teamcode.drive.robo9u.Modules.Mechanisms;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(group="Demo", preselectTeleOp = "localiz_rami_field_centric")
public class LeftMidMarkers extends LinearOpMode {
    Mechanisms mecanisme;
    SampleMecanumDrive drive;
    Detection detection;
    TrajectorySequence fullTrajectory;
    Trajectory[] gotoPark = new Trajectory[3];
    public static double corectieeroarex = 1.9, corectieeroarey = 0.45, gripTime = 350;
    int conesPlaced = 0;
    private boolean parking = false;

    private ElapsedTime runtime;

    public void initialize()
    {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.imu.startImuThread(this);
        mecanisme = new Mechanisms(hardwareMap);
        detection = new Detection(hardwareMap, "Webcam 0");
        runtime = new ElapsedTime();
        detection.setSleeveDetectionMode();
        fullTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
                .splineToSplineHeading(new Pose2d(40, 4, Math.toRadians(-90)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(52.5, -8.5, Math.toRadians(-180)), Math.toRadians(-90))
                .addDisplacementMarker(()->drive.setPoseEstimate(drive.getPoseEstimate().plus(new Pose2d(corectieeroarex, corectieeroarey, 0))))
                .UNSTABLE_addTemporalMarkerOffset(-0.25, ()->mecanisme.lift.nextStack())
                .lineToSplineHeading(new Pose2d(52.5, 28.5, Math.toRadians(-270)))
                .UNSTABLE_addTemporalMarkerOffset(-0.25, ()->mecanisme.claw.Close())
                .UNSTABLE_addTemporalMarkerOffset(0, ()->mecanisme.lift.setLiftState(Lift.LiftState.Mid))
                .lineToSplineHeading(new Pose2d(52.5, -8.5, Math.toRadians(-180)))
                .addDisplacementMarker(()->drive.setPoseEstimate(drive.getPoseEstimate().plus(new Pose2d(corectieeroarex, corectieeroarey, 0))))
                .UNSTABLE_addTemporalMarkerOffset(-0.25, ()->mecanisme.lift.nextStack())
                .lineToSplineHeading(new Pose2d(52.5, 28.5, Math.toRadians(-270)))
                .UNSTABLE_addTemporalMarkerOffset(-0.25, ()->mecanisme.claw.Close())
                .UNSTABLE_addTemporalMarkerOffset(0, ()->mecanisme.lift.setLiftState(Lift.LiftState.Mid))
                .lineToSplineHeading(new Pose2d(52.5, -8.5, Math.toRadians(-180)))
                .addDisplacementMarker(()->drive.setPoseEstimate(drive.getPoseEstimate().plus(new Pose2d(corectieeroarex, corectieeroarey, 0))))
                .UNSTABLE_addTemporalMarkerOffset(-0.25, ()->mecanisme.lift.nextStack())
                .lineToSplineHeading(new Pose2d(52.5, 28.5, Math.toRadians(-270)))
                .UNSTABLE_addTemporalMarkerOffset(-0.25, ()->mecanisme.claw.Close())
                .UNSTABLE_addTemporalMarkerOffset(0, ()->mecanisme.lift.setLiftState(Lift.LiftState.Mid))
                .lineToSplineHeading(new Pose2d(52.5, -8.5, Math.toRadians(-180)))
                .addDisplacementMarker(()->drive.setPoseEstimate(drive.getPoseEstimate().plus(new Pose2d(corectieeroarex, corectieeroarey, 0))))
                .UNSTABLE_addTemporalMarkerOffset(-0.25, ()->mecanisme.lift.nextStack())
                .lineToSplineHeading(new Pose2d(52.5, 28.5, Math.toRadians(-270)))
                .UNSTABLE_addTemporalMarkerOffset(-0.25, ()->mecanisme.claw.Close())
                .UNSTABLE_addTemporalMarkerOffset(0, ()->mecanisme.lift.setLiftState(Lift.LiftState.Mid))
                .lineToSplineHeading(new Pose2d(52.5, -8.5, Math.toRadians(-180)))
                .addDisplacementMarker(()->drive.setPoseEstimate(drive.getPoseEstimate().plus(new Pose2d(corectieeroarex, corectieeroarey, 0))))
                .UNSTABLE_addTemporalMarkerOffset(-0.25, ()->mecanisme.lift.nextStack())
                .lineToSplineHeading(new Pose2d(52.5, 28.5, Math.toRadians(-270)))
                .UNSTABLE_addTemporalMarkerOffset(-0.25, ()->mecanisme.claw.Close())
                .UNSTABLE_addTemporalMarkerOffset(0, ()->mecanisme.lift.setLiftState(Lift.LiftState.Mid))
                .lineToSplineHeading(new Pose2d(52.5, -8.5, Math.toRadians(-180)))
                .addDisplacementMarker(()->drive.setPoseEstimate(drive.getPoseEstimate().plus(new Pose2d(corectieeroarex, corectieeroarey, 0))))
                .build();

        gotoPark[0] = drive.trajectoryBuilder(fullTrajectory.end()).lineToLinearHeading(new Pose2d(52.5, 28, Math.toRadians(-180))).build();
        gotoPark[1] = drive.trajectoryBuilder(fullTrajectory.end()).lineToLinearHeading(new Pose2d(52.5,  -2, Math.toRadians(-180))).build();
        gotoPark[2] = drive.trajectoryBuilder(fullTrajectory.end()).lineToLinearHeading(new Pose2d(52.5, -24, Math.toRadians(-180))).build();

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
        PhotonCore.CONTROL_HUB.clearBulkCache();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        detection.stopCamera();
        runtime.reset();
        drive.followTrajectorySequenceAsync(fullTrajectory);
        while(!isStopRequested() && opModeIsActive()) {
            PhotonCore.CONTROL_HUB.clearBulkCache();

            if(drive.isBusy() && !parking) {
                drive.followTrajectoryAsync(gotoPark[detection.getParkingIndex()]);
                parking = true;
            }
            mecanisme.update();
            drive.update();
            telemetry.addLine(conesPlaced + " cones placed");
            telemetry.addLine("Running at " + 1e9/runtime.nanoseconds() + "hz");
            runtime.reset();
            telemetry.update();
        }
        mecanisme.lift.stopCurrentTrajectory();
        drive.breakFollowing();
        SampleMecanumDrive.lastAutonomousPosition = drive.getPoseEstimate();
    }
}