package org.firstinspires.ftc.teamcode.drive.robo9u.teleops;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.robo9u.Modules.Detection;
import org.firstinspires.ftc.teamcode.drive.robo9u.Modules.Lift;
import org.firstinspires.ftc.teamcode.drive.robo9u.Modules.Mechanisms;

@TeleOp(name="Normal Field-Centric")
public class NormalFieldCentric extends LinearOpMode {

    private SampleMecanumDrive drive;
    private Mechanisms mecanisme;
    private ElapsedTime runtime;

    double drivepow = 0.8;

    public void initialize() 
    {
        

        drive = new SampleMecanumDrive(hardwareMap);
        drive.imu.startImuThread(this);
        mecanisme = new Mechanisms(hardwareMap);
        runtime = new ElapsedTime();

        drive.setPoseEstimate(SampleMecanumDrive.lastAutonomousPosition);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mecanisme.lift.fourBar.down();
        mecanisme.claw.Open();

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
        PhotonCore.CONTROL_HUB.clearBulkCache();
    }

    public void updateDrivePowers()
    {
        if(gamepad1.left_stick_button) drive.setPoseEstimate(new Pose2d());
        double LF, RF, LR, RR;
        double forward, rotate, strafe, denominator, theta;
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
        drive.setMotorPowers(LF, LR, RR, RF);
    }

    public void updateClaw()
    {
        if(gamepad2.b) {
            mecanisme.claw.Close();
        }else if(gamepad2.x){
            mecanisme.claw.Open();
        }
        if(gamepad2.a){
            mecanisme.lift.fourBar.down();
        }else if(gamepad2.y){
            mecanisme.lift.fourBar.up();
        }
    }
    public void updateLift() {
        mecanisme.lift.setPower(gamepad1.left_trigger - gamepad1.right_trigger);

        if (gamepad2.dpad_up){ // auto control
            mecanisme.lift.setLiftState(Lift.LiftState.High);
        }else if(gamepad2.dpad_left || gamepad2.dpad_right){
            mecanisme.lift.setLiftState(Lift.LiftState.Mid);
        }else if(gamepad2.dpad_down){
            mecanisme.lift.setLiftState(Lift.LiftState.Low);
        }else if(gamepad2.left_bumper){
            mecanisme.lift.setLiftState(Lift.LiftState.Ground);
        }else if(gamepad2.right_bumper){
            mecanisme.lift.stopCurrentTrajectory();
        }
    }

    public void updatetelemetry(){
        telemetry.addLine("Running at " + 1e9/runtime.nanoseconds() + "hz");
        runtime.reset();
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        while(!isStopRequested() && opModeIsActive())
        {
            PhotonCore.CONTROL_HUB.clearBulkCache();
            updateDrivePowers();
            updateClaw();
            updateLift();
            mecanisme.update();
            drive.update();
            updatetelemetry();
        }
    }
}