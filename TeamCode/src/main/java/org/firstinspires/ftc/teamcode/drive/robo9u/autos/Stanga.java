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

    public enum RobotState{
        ROBOT_INIT, // robot -> peste junction cu preload
        ROBOT_PLACE, // robot -> pune con si se duce la stack
        ROBOT_STACK, // robot -> ia din stack si se duce la junction sau parcheaza
        ROBOT_FINISH
    }
    RobotState currentState = RobotState.ROBOT_INIT;

    Mechanisms mecanisme;
    SampleMecanumDrive drive;
    Detection detection;

    TrajectorySequence gotoMidfromPreload;
    Trajectory gotoStackfromMid, gotoMidfromStack;
    Trajectory[] gotoPark = new Trajectory[3];
    public static double corectieeroarex = 1.45, corectieeroarey = 0;

    public static double gripTime = 350;

    int conesPlaced = 1;

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

        gotoMidfromPreload = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(40, 4, Math.toRadians(-90)))
                .splineTo(new Vector2d(53, -10), Math.toRadians(-90))
                .build();
        gotoStackfromMid = drive.trajectoryBuilder(gotoMidfromPreload.end())
                .lineToSplineHeading(new Pose2d(53, 28.5, Math.toRadians(-270)))
                .build();
        gotoMidfromStack = drive.trajectoryBuilder(gotoStackfromMid.end())
                .lineToSplineHeading(new Pose2d(53, -10, Math.toRadians(-180)))
                .build();

        gotoPark[0] = drive.trajectoryBuilder(gotoMidfromStack.end()).lineToLinearHeading(new Pose2d(53, 23, Math.toRadians(-180))).build();
        gotoPark[1] = drive.trajectoryBuilder(gotoMidfromStack.end()).lineToLinearHeading(new Pose2d(53,  -2, Math.toRadians(-180))).build();
        gotoPark[2] = drive.trajectoryBuilder(gotoMidfromStack.end()).lineToLinearHeading(new Pose2d(53, -624, Math.toRadians(-180))).build();
    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();
        detection.stopCamera();
        runtime.reset();
        timer.reset();
        while(!isStopRequested() && opModeIsActive()) {
            switch (currentState) {
                case ROBOT_INIT:
                    mecanisme.claw.Close();
                    if(timer.milliseconds() >= gripTime){
                        mecanisme.lift.setLiftState(Lift.LiftState.Mid);
                        drive.followTrajectorySequenceAsync(gotoMidfromPreload);
                        currentState = RobotState.ROBOT_PLACE;
                        timer.reset();
                    }
                    break;
                case ROBOT_PLACE:
                    if(!drive.isBusy()) {
                        mecanisme.claw.Open();
                        mecanisme.lift.fourBar.down();
                        if (timer.milliseconds() >= gripTime) {
                            conesPlaced += 1;
                            drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX() + corectieeroarex, drive.getPoseEstimate().getY() + corectieeroarey, drive.getPoseEstimate().getHeading()));
                            if(conesPlaced < 6){
                                drive.followTrajectoryAsync(gotoStackfromMid);
                                mecanisme.lift.nextStack();
                                currentState = RobotState.ROBOT_STACK;
                            }else{
                                mecanisme.lift.setLiftState(Lift.LiftState.Ground);
                                drive.followTrajectoryAsync(gotoPark[detection.getParkingIndex()]);
                                currentState = RobotState.ROBOT_FINISH;
                            }
                            timer.reset();
                        }
                    }else{
                        timer.reset();
                    }
                    break;
                case ROBOT_STACK:
                    if(!drive.isBusy()){
                        mecanisme.claw.Close();
                        if(timer.milliseconds() >= gripTime){
                            drive.followTrajectoryAsync(gotoMidfromStack);
                            mecanisme.lift.setLiftState(Lift.LiftState.Mid);
                            currentState = RobotState.ROBOT_PLACE;
                            timer.reset();
                        }
                    }else{
                        timer.reset();
                    }
                    break;
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
        SampleMecanumDrive.lastAutonomousPosition = drive.getPoseEstimate();
    }
}