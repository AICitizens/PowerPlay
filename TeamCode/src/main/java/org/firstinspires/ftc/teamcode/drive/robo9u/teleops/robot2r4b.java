package org.firstinspires.ftc.teamcode.drive.robo9u.teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
@TeleOp(name="robot_2r4b")
    public class robot2r4b extends LinearOpMode {

    private SampleMecanumDrive drive;
    double drivepow = 1;

    public void initialize() {
        drive = new SampleMecanumDrive(hardwareMap, this);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void updateDrivePowers() {
        double LF, RF, LR, RR;
        double forward, rotate, strafe, denominator;
        rotate = gamepad1.right_stick_x;
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        denominator = Math.max(1, Math.abs(strafe) + Math.abs(forward) + Math.abs(rotate));
        LF = (strafe + forward + rotate) / denominator;
        RF = (-strafe + forward - rotate) / denominator;
        LR = (-strafe + forward + rotate) / denominator;
        RR = (strafe + forward - rotate) / denominator;

        drive.setMotorPowers(LF, RF, LR, RR);
    }

    public void setMotorPowers(double v, double v1, double v2, double v3) {
        drive.leftFront.setPower(v);
        drive.leftRear.setPower(v1);
        drive.rightRear.setPower(v2);
        drive.rightFront.setPower(v3);


    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            updateDrivePowers();

            drive.update();
        }
    }
}

