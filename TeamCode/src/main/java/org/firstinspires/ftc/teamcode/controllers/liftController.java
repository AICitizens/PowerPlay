package org.firstinspires.ftc.teamcode.controllers;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class liftController {
    public static final double TICKS_PER_REV = 537.7;
    public static final double MAX_RPM = 312;
    public static double WHEEL_RADIUS = 3.565; // cm
    public static double GEAR_RATIO = 0.5; // output (wheel) speed / input (motor) speed
    public static double kp = 0, ki = 0, kd = 0;

    private Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();

    public DcMotorEx left, right;

    public static double target = 0; //cm

    public liftController(HardwareMap hw) {
        left = hw.get(DcMotorEx.class, "liftLeft");
        right = hw.get(DcMotorEx.class, "liftRight");

        left.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public double getCurrentPosition(){
        return (left.getCurrentPosition()+right.getCurrentPosition())/2;
    }



    public void update(){
        double leftPower, rightPower;
        double error = Math.abs(encoderTicksToCM(target) - getCurrentPosition())>10?encoderTicksToCM(target) - getCurrentPosition():0;
        double motorRelativeError = Math.abs(left.getCurrentPosition()-right.getCurrentPosition())>10?left.getCurrentPosition()-right.getCurrentPosition():0;
        leftPower = kp*error-2*kp*motorRelativeError;
        rightPower = kp*error+2*kp*motorRelativeError;
        double denom = Math.max(leftPower, Math.max(rightPower, 1));
        left.setPower(leftPower / denom);
        right.setPower(rightPower / denom);
        telemetry.addData("error", error);
        telemetry.addData("relativeError", motorRelativeError);
        telemetry.addData("leftPos", left.getCurrentPosition());
        telemetry.addData("rightPos", right.getCurrentPosition());
        telemetry.update();
    }

    public void setTarget(double newTarget){
        target = newTarget;
    }

    public static double encoderTicksToCM(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
}
