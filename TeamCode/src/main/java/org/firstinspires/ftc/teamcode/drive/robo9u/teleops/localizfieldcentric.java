package org.firstinspires.ftc.teamcode.drive.robo9u.teleops;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.drive.SampleLiftController;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.robo9u.Mecanisme.Mechanisms;

@TeleOp(name="localiz_field_centric")
public class localizfieldcentric extends LinearOpMode {

    SampleMecanumDrive drive;
    Mechanisms mecanisme;

    public void initialize() 
    {
        drive = new SampleMecanumDrive(hardwareMap);
        mecanisme = new Mechanisms(hardwareMap);
    }

    public void updateDrivePowers()
    {
        if(gamepad1.right_bumper){
            drive.setPoseEstimate(new Pose2d());
        }

        double LF, RF, LR, RR;
        double forward, rotate, strafe, denominator, theta;
        double drivepow = 0.8;
        theta = drive.getPoseEstimate().getHeading();

        rotate=gamepad1.right_stick_x;
        forward=-gamepad1.left_stick_y;
        strafe=gamepad1.left_stick_x;
        double oldfw = forward;
        double oldstr = strafe;
        forward = oldfw * Math.cos(theta) - oldstr * Math.sin(theta);
        strafe = oldfw * Math.sin(theta) + oldstr * Math.cos(theta);

        denominator = Math.max(1,Math.abs(forward)+Math.abs(strafe)+Math.abs(rotate)) ;
        LF = (strafe+forward+rotate)/denominator*drivepow;
        RF = (-strafe+forward-rotate)/denominator*drivepow;
        LR = (-strafe+forward+rotate)/denominator*drivepow;
        RR = (strafe+forward-rotate)/denominator*drivepow;
        setMotorPowers(LF, LR, RR, RF);
    }

    public void setMotorPowers(double v, double v1, double v2, double v3) {
        drive.leftFront.setPower(v);
        drive.leftRear.setPower(v1);
        drive.rightRear.setPower(v2);
        drive.rightFront.setPower(v3);
    }

    public void updateClaw()
    {
        if(gamepad2.b) {
            mecanisme.claw.Close();
        }else if(gamepad2.x){
            mecanisme.claw.Open();
        }
        if(gamepad2.a){
            mecanisme.lift.parallelo.down();
        }else if(gamepad2.y){
            mecanisme.lift.parallelo.up();
        }
    }
    public void updateLift() {
        mecanisme.lift.setPower(gamepad1.left_trigger - gamepad1.right_trigger);

        if (gamepad2.dpad_up){ // auto control
            mecanisme.lift.goToHigh();
        }else if(gamepad2.dpad_left || gamepad2.dpad_right){
            mecanisme.lift.goToMid();
        }else if(gamepad2.dpad_down){
            mecanisme.lift.goToLow();
        }else if(gamepad2.left_bumper){
            mecanisme.lift.retractFully();
        }else if(gamepad2.right_bumper){
            mecanisme.lift.stopCurrentTrajectory();
        }
    }

    public void updatetelemetry(){
        telemetry.addData("x", drive.getPoseEstimate().getX());
        telemetry.addData("y", drive.getPoseEstimate().getY());
        telemetry.addData("heading", drive.getPoseEstimate().getHeading());
        telemetry.addData("lift", mecanisme.lift.lift.getPoseEstimate().getX());
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        mecanisme.lift.parallelo.down();
        mecanisme.claw.Open();
        while(!isStopRequested())
        {
            updateDrivePowers();
            updateClaw();
            updateLift();
            mecanisme.update();
            drive.update();
            updatetelemetry();
        }
    }
}