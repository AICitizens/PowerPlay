package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
@TeleOp(name = "hammerhead")
 public class hammerhead extends LinearOpMode {

    private DcMotor stangaspateMotor;
    private DcMotor stangafataMotor;
    private DcMotor dreaptaspateMotor;
    private DcMotor dreaptafataMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        stangafataMotor = hardwareMap.get(DcMotor.class, "left1_motor");
        stangaspateMotor = hardwareMap.get(DcMotor.class, "left2_motor");
        dreaptafataMotor = hardwareMap.get(DcMotor.class, "right1_motor");
        dreaptaspateMotor = hardwareMap.get(DcMotor.class, "right2_motor");



        stangaspateMotor.setDirection(DcMotor.Direction.REVERSE);
        stangafataMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive()) {

            double leftPower = -gamepad1.left_stick_y;
            double rightPower = -gamepad1.right_stick_y;


            stangafataMotor.setPower(leftPower);
            stangaspateMotor.setPower(leftPower);
            dreaptaspateMotor.setPower(rightPower);
            dreaptafataMotor.setPower(rightPower);

            idle();
        }
    }
}

