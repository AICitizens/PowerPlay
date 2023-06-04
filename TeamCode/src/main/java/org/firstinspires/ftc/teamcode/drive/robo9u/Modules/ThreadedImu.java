package org.firstinspires.ftc.teamcode.drive.robo9u.Modules;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ThreadedImu {

    public IMU imu;
    private Object imuLock = new Object();

    public double imuYaw;
    public double imuYawVelocity;


    public ThreadedImu(HardwareMap hw){
        imu = hw.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        imu.resetYaw();
    }

    public void startImuThread(LinearOpMode opMode){
        new Thread(()->{
            while(!opMode.isStopRequested() && opMode.opModeIsActive())
           synchronized (imuLock){
               imuYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
               imuYawVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
           }
        });
    }
    public double getHeading(){
        return imuYaw;
    }
    public double getHeadingVelocity(){
        return imuYawVelocity;
    }
}
