package org.firstinspires.ftc.teamcode.drive.robo9u.teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.robo9u.Mecanisme.Mechanisms;
@Disabled
@TeleOp(name="localiz_1_driver")
public class test1driver extends LinearOpMode {

    SampleMecanumDrive drive;
    Mechanisms mecanisme;

    public void initialize()
    {
        drive = new SampleMecanumDrive(hardwareMap);
        mecanisme = new Mechanisms(hardwareMap);
    }

    public void updateDrivePowers()
    {
        double LF, RF, LR, RR;
        double forward, rotate, strafe, denominator;
        double drivepow;
        drivepow = (gamepad1.right_trigger > 0 || gamepad1.right_bumper) ? 0.6 : 0.8;
        rotate=gamepad1.right_stick_x;
        forward=-gamepad1.left_stick_y*1.1;
        strafe=gamepad1.left_stick_x;
        denominator = Math.max(1,Math.abs(strafe)+Math.abs(forward)+Math.abs(rotate)) ;
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
        if(gamepad1.b) {
            mecanisme.claw.Close();
        }else if(gamepad1.x){
            mecanisme.claw.Open();
        }
        if(gamepad1.a){
            mecanisme.lift.parallelo.down();
        }else if(gamepad1.y) {
            mecanisme.lift.parallelo.up();
        }
    }

    public void updateLift() {
        mecanisme.lift.setPower(gamepad1.left_trigger - gamepad1.right_trigger);

        if (gamepad1.dpad_up){ // auto control
            mecanisme.lift.goToHigh();
        }else if(gamepad1.dpad_left || gamepad1.dpad_right){
            mecanisme.lift.goToHigh();
        }else if(gamepad1.dpad_down){
            mecanisme.lift.goToHigh();
        }else if(gamepad1.left_bumper){
            mecanisme.lift.retractFully();
        }
    }
    public void updatetelemetry(){
        telemetry.addData("x", drive.getPoseEstimate().getX());
        telemetry.addData("y", drive.getPoseEstimate().getY());
        telemetry.addData("heading", drive.getPoseEstimate().getHeading());
        telemetry.addData("lift", mecanisme.lift.lift.getCurrentPosition());
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