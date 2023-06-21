package org.firstinspires.ftc.teamcode.drive.robo9u.Modules;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ThreadedIMU {

    private final Object imuLock = new Object();
    private final IMU imu;

    private double imuYaw;
    private double imuYawVelocity;

    public ThreadedIMU(HardwareMap hardwareMap){
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
    }
    public void startImuThread(LinearOpMode opMode){
        synchronized (imuLock){
            new Thread(()->{
                opMode.waitForStart();//just in case the thread is started before opMode start
                while(opMode.opModeIsActive() && !opMode.isStopRequested()) {
                    imuYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                    imuYawVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
                }
            }).start();
        }
    }
    public double getYaw(){
        return imuYaw;
    }
    public double getYawVelocity(){
        return imuYawVelocity;
    }
}