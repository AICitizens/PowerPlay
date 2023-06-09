package org.firstinspires.ftc.teamcode.drive;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.List;

@Config
public class LiftController {
    private PIDFController controller;

    public static final double TICKS_PER_REV = 537.7;
    public static double WHEEL_RADIUS = 3.565/2; // cm
    public static double GEAR_RATIO = 1.64; // output (wheel) speed / input (motor) speed
    public static double kp = 0.01, ki = 0, kd = 0, ff = 0.000035, relativeP = 0.0005;
    public double target = 0; //ticks

    private boolean canOverride = true;

    private Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();

    public DcMotorEx left, right;
    private List<DcMotorEx> motors;

    public LiftController(HardwareMap hw, boolean resetEncoders) {
        controller = new PIDFController(kp, ki, kd, ff);
        controller.setTolerance(30);

        left = hw.get(DcMotorEx.class, "liftLeft");
        right = hw.get(DcMotorEx.class, "liftRight");

        motors = Arrays.asList(left, right);

        for(DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        left.setDirection(DcMotorEx.Direction.REVERSE);

        if(!resetEncoders) return;
        for(DcMotorEx motor : motors) {
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public boolean isBusy(){
        return !canOverride;
    }

    public double getCurrentPosition(){
        return ((float)left.getCurrentPosition()+(float)right.getCurrentPosition())/2;
    }

    public void setPower(double power) {
        left.setPower(power);
        right.setPower(power);
    }

    public void stop(){
        target = getCurrentPosition();
    }

    public void update(){
        controller.setPIDF(kp, ki, kd, ff);
        double leftPower, rightPower;
        double motorRelativeError = Math.abs(left.getCurrentPosition()-right.getCurrentPosition())>10?left.getCurrentPosition()-right.getCurrentPosition():0;
        double power = controller.calculate(getCurrentPosition(), target);
        leftPower = power-relativeP*motorRelativeError;
        rightPower = power+relativeP*motorRelativeError;
        double denom = Math.max(leftPower, Math.max(rightPower, 1));
        left.setPower(leftPower / denom);
        right.setPower(rightPower / denom);
        telemetry.addData("relativeError", motorRelativeError);
        telemetry.addData("error", controller.getPositionError());
        telemetry.addData("target", controller.getSetPoint());
        telemetry.addData("power", power);
        telemetry.addData("leftPos", left.getCurrentPosition());
        telemetry.addData("rightPos", right.getCurrentPosition());
        if(controller.atSetPoint()){
            canOverride = true;
        }
        telemetry.update();
    }

    public void setTarget(double newTarget){
        target = newTarget * TICKS_PER_REV / WHEEL_RADIUS / 2 / Math.PI / GEAR_RATIO;
        canOverride = false;
    }

    public static double encoderTicksToCM(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
}
