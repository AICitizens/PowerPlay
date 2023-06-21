package org.firstinspires.ftc.teamcode.drive.robo9u.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.robo9u.Modules.Detection;
import org.firstinspires.ftc.teamcode.drive.robo9u.Modules.Lift;
import org.firstinspires.ftc.teamcode.drive.robo9u.Modules.Mechanisms;

@TeleOp(name="Normal Robot-Centric")
public class NormalRobotCentric extends LinearOpMode {

    private SampleMecanumDrive drive;
    private Mechanisms mecanisme;
    private ElapsedTime runtime;

    double drivepow = 0.8;

    public void initialize() 
    {
        

        drive = new SampleMecanumDrive(hardwareMap, this);
        mecanisme = new Mechanisms(hardwareMap);
        runtime = new ElapsedTime();

        drive.setPoseEstimate(SampleMecanumDrive.lastAutonomousPosition);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mecanisme.lift.fourBar.down();
        mecanisme.claw.Open();
    }

    public void updateDrivePowers()
    {
        double LF, RF, LR, RR;
        double x, y, x2, denominator;
        drivepow = (gamepad1.right_trigger > 0 || gamepad1.right_bumper)?0.6:0.8;
        x2=gamepad1.right_stick_x;
        y=-gamepad1.left_stick_y*1.1;
        x=gamepad1.left_stick_x;
        denominator = Math.max(1,Math.abs(x)+Math.abs(y)+Math.abs(x2)) ;
        LF = (x+y+x2)/denominator*drivepow;
        RF = (-x+y-x2)/denominator*drivepow;
        LR = (-x+y+x2)/denominator*drivepow;
        RR = (x+y-x2)/denominator*drivepow;
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
            updateDrivePowers();
            updateClaw();
            updateLift();
            mecanisme.update();
            drive.update();
            updatetelemetry();
        }
    }
}