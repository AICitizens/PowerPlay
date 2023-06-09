package org.firstinspires.ftc.teamcode.drive.robo9u.teleops;

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

@TeleOp(name="Rami Robot-Centric")
public class RamiRobotCentric extends LinearOpMode {

    private SampleMecanumDrive drive;
    private Mechanisms mecanisme;
    private ElapsedTime runtime;

    Boolean lastbutton = false;
    double drivepow = 1;

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
        double LF, RF, LR, RR;
        double forward, rotate, strafe, denominator;
        if(gamepad1.right_stick_button && !lastbutton) drivepow = (drivepow == 0.8)?0.6:0.8;
        lastbutton = gamepad1.right_stick_button;
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
        if(gamepad1.right_bumper) {
            mecanisme.claw.Close();
        }else if(gamepad1.left_bumper){
            mecanisme.claw.Open();
        }
        if(gamepad1.dpad_down){
            mecanisme.lift.fourBar.down();
        }else if(gamepad1.dpad_up) {
            mecanisme.lift.fourBar.up();
        }
        if(gamepad1.dpad_left || gamepad1.dpad_right){
            mecanisme.claw.dropConeAndKeepBeacon();
        }
    }

    public void updateLift() {
        mecanisme.lift.setPower(gamepad1.left_trigger - gamepad1.right_trigger);

        if (gamepad1.a){ // auto control
            mecanisme.lift.setLiftState(Lift.LiftState.High);
        }else if(gamepad1.y){
            mecanisme.lift.setLiftState(Lift.LiftState.Mid);
        }else if(gamepad1.b){
            mecanisme.lift.setLiftState(Lift.LiftState.Low);
        }else if(gamepad1.x){
            mecanisme.lift.setLiftState(Lift.LiftState.Ground);
            mecanisme.claw.Open();
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