package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class motortest extends LinearOpMode {
    public static double DISTANCE = 60; // in

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive 

drive = new SampleMecanumDrive(hardwareMap);;;

        waitForStart();

        if (isStopRequested()) return;

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested()) {
            drive.leftFront.setPower(1);
            sleep(1000);
            drive.leftFront.setPower(0);
            sleep(1000);
            drive.rightFront.setPower(1);
            sleep(1000);
            drive.rightFront.setPower(0);
            sleep(1000);
            drive.leftRear.setPower(1);
            sleep(1000);
            drive.leftRear.setPower(0);
            sleep(1000);
            drive.rightRear.setPower(1);
            sleep(1000);
            drive.rightRear.setPower(0);
            sleep(1000);
        };
    }
}
