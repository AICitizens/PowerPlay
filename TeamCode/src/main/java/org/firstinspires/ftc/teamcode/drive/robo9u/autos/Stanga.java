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

    private enum RobotState{
        ROBOT_INIT, // robot -> peste junction cu preload
        ROBOT_PLACE, // robot -> pune con si se duce la stack
        ROBOT_STACK, // robot -> ia din stack si se duce la junction sau parcheaza
        ROBOT_FINISH
    }

    private Mechanisms mecanisme;
    private SampleMecanumDrive drive;
    private Detection detection;

    private TrajectorySequence gotoMidfromPreload;
    private Trajectory gotoStackfromMid, gotoMidfromStack;
    private Trajectory[] gotoPark = new Trajectory[3];

    public static double preloadPoseX = 41.5, preloadPoseY = 4.5, preloadPoseHeading = -90;
    public static double junctionPoseX = 53, junctionPoseY = -6.5, junctionPoseHeading = -170;
    public static double stackPoseX = 53.5, stackPoseY = 30.5, stackPoseHeading = -270;
    public static double corectieeroarex = 1.5, corectieeroarey = 0.35;
    public static int gripTime = 300;

    RobotState currentState;

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
        currentState = RobotState.ROBOT_INIT;
        drive.setPoseEstimate(new Pose2d(0, 0, 0)); //0.5 la natio

        gotoMidfromPreload = drive.trajectorySequenceBuilder(new Pose2d())
                .splineToSplineHeading(new Pose2d(preloadPoseX, preloadPoseY, Math.toRadians(preloadPoseHeading)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(junctionPoseX, junctionPoseY, Math.toRadians(junctionPoseHeading)), Math.toRadians(-90)).build();
        gotoStackfromMid = drive.trajectoryBuilder(gotoMidfromPreload.end()).lineToLinearHeading(new Pose2d(stackPoseX, stackPoseY, Math.toRadians(stackPoseHeading))).build();
        gotoMidfromStack = drive.trajectoryBuilder(gotoStackfromMid.end()).lineToLinearHeading(new Pose2d(junctionPoseX, junctionPoseY, Math.toRadians(junctionPoseHeading))).build();
        gotoPark[0] = drive.trajectoryBuilder(gotoMidfromStack.end()).lineToLinearHeading(new Pose2d(52, -23, Math.toRadians(-270))).build();
        gotoPark[1] = drive.trajectoryBuilder(gotoMidfromStack.end()).lineToLinearHeading(new Pose2d(50,  0, Math.toRadians(0))).build();
        gotoPark[2] = drive.trajectoryBuilder(gotoMidfromStack.end()).lineToLinearHeading(new Pose2d(52.5, 24, Math.toRadians(-270))).build();
    }


    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();
        parkingIndex = detection.getParkingIndex();
        timer.reset();
        while(!isStopRequested() && opModeIsActive()) {
            runtime.reset();
            switch (currentState) {
                case ROBOT_INIT:
                    mecanisme.claw.Close();
                    mecanisme.lift.setLiftState(Lift.LiftState.Mid);
                    drive.followTrajectorySequenceAsync(gotoMidfromPreload);
                    currentState = RobotState.ROBOT_PLACE;
                    break;
                case ROBOT_PLACE:
                    if(!mecanisme.lift.lift.isBusy() && !drive.isBusy()) {
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
                                drive.followTrajectoryAsync(gotoPark[parkingIndex]);
                                currentState = RobotState.ROBOT_FINISH;
                            }
                            timer.reset();
                        }
                    }else{
                        timer.reset();
                    }
                    break;
                case ROBOT_STACK:
                    if(!drive.isBusy() && !mecanisme.lift.lift.isBusy()){
                        mecanisme.claw.Close();
                        if(timer.milliseconds() >= gripTime){
                            mecanisme.lift.fourBar.servo.setPosition(0.6);
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
            telemetry.addLine("Running at " + 1e6/runtime.nanoseconds() + "hz");
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