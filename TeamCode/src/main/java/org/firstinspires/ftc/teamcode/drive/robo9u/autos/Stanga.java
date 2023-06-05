package org.firstinspires.ftc.teamcode.drive.robo9u.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.robo9u.Modules.Lift;
import org.firstinspires.ftc.teamcode.drive.robo9u.Modules.Mechanisms;
import org.firstinspires.ftc.teamcode.drive.robo9u.Modules.Detection;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(group="Demo", preselectTeleOp = "localiz_rami_field_centric")
public class Stanga extends LinearOpMode {

    private Mechanisms mecanisme;
    private SampleMecanumDrive drive;
    private Detection detection;
    private TrajectorySequence full;
    private Trajectory[] gotoPark = new Trajectory[3];
    public static double corectieeroarex = 1.5, corectieeroarey = 0.35;
    public static int gripTime = 300;
    boolean hasParked = false;

    int conesPlaced = 0;
    int parkingIndex = 1;

    private ElapsedTime timer;
    private ElapsedTime runtime;

    public void initialize()
    {
        drive = new SampleMecanumDrive(hardwareMap);
        mecanisme = new Mechanisms(hardwareMap);
        detection = new Detection(hardwareMap, "Webcam 0");
        runtime = new ElapsedTime();
        timer = new ElapsedTime();

        detection.setSleeveDetectionMode();



        full = drive.trajectorySequenceBuilder(new Pose2d(-34.58, -62.28, Math.toRadians(89.57)))
                .splineTo(new Vector2d(-29.05, -9.54), Math.toRadians(62.18))
                .addSpatialMarker(new Vector2d(-29.05, -9.54), ()->mecanisme.lift.setLiftState(Lift.LiftState.Ground))
                .waitSeconds(1)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-36.47, -15.22, Math.toRadians(171.33)), Math.toRadians(171.33))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-59.92, -12.01, Math.toRadians(179.14)), Math.toRadians(179.14))
                .addSpatialMarker(new Vector2d(-59.92, -12.01), ()->{mecanisme.claw.Close();  mecanisme.lift.setLiftState(Lift.LiftState.High);})
                .waitSeconds(1)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-27.88, -9.25, Math.toRadians(59.66)), Math.toRadians(59.66))
                .addSpatialMarker(new Vector2d(-27.88, -9.25), ()->mecanisme.lift.setLiftState(Lift.LiftState.Ground))
                .waitSeconds(1)
                .setReversed(false)
                .build();
        drive.setPoseEstimate(full.start());
        gotoPark[0] = drive.trajectoryBuilder(full.end()).lineToLinearHeading(new Pose2d(52, -23, Math.toRadians(-270))).build();
        gotoPark[1] = drive.trajectoryBuilder(full.end()).lineToLinearHeading(new Pose2d(50,  0, Math.toRadians(0))).build();
        gotoPark[2] = drive.trajectoryBuilder(full.end()).lineToLinearHeading(new Pose2d(52.5, 24, Math.toRadians(-270))).build();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        mecanisme.claw.Close();
        sleep(250);
        mecanisme.lift.setLiftState(Lift.LiftState.High);
        drive.followTrajectorySequenceAsync(full);
        parkingIndex = detection.getParkingIndex();
        while(!isStopRequested() && opModeIsActive()) {
            if(!drive.isBusy() && !hasParked){
                hasParked = true;
                //drive.followTrajectoryAsync(gotoPark[parkingIndex]);
            }
            telemetry.addLine("Running at " + 1e9/runtime.nanoseconds() + "hz");
            runtime.reset();
            telemetry.addLine(conesPlaced + " cones placed");
            telemetry.update();
            mecanisme.update();
            drive.update();
        }
        mecanisme.lift.stopCurrentTrajectory();
        drive.breakFollowing();
        detection.stopCamera();
        SampleMecanumDrive.lastAutonomousPosition = drive.getPoseEstimate();
    }
}