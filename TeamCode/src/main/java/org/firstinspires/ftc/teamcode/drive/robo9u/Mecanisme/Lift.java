package org.firstinspires.ftc.teamcode.drive.robo9u.Mecanisme;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.drive.SampleLiftController;

import java.util.HashMap;

@Config
public class Lift {
    TouchSensor liftSensor;
    public SampleLiftController lift;
    public Parallelo parallelo;
    public boolean canStop = false;
    public static double ground = 0, low = 9, mid = 32, high = 58, stackConeDist = 3.25, stackPos;

    Trajectory currentTrajectory;

    public void stopCurrentTrajectory(){
        lift.breakFollowing();
        canStop = false;
    }

    public void goToHigh(){
        stopCurrentTrajectory();
        canStop = false;
        currentTrajectory = lift.trajectoryBuilder(lift.getPoseEstimate()).lineTo(new Vector2d(high,0)).build();
        lift.followTrajectoryAsync(currentTrajectory);
        parallelo.up();
    }

    public void goToMid(){
        stopCurrentTrajectory();
        canStop = false;
        currentTrajectory = lift.trajectoryBuilder(lift.getPoseEstimate()).lineTo(new Vector2d(mid,0)).build();
        lift.followTrajectoryAsync(currentTrajectory);
        parallelo.up();
    }

    public void goToLow(){
        stopCurrentTrajectory();
        canStop = false;
        currentTrajectory = lift.trajectoryBuilder(lift.getPoseEstimate()).lineTo(new Vector2d(low,0)).build();
        lift.followTrajectoryAsync(currentTrajectory);
        parallelo.up();
    }

    public void retractFully(){
        stopCurrentTrajectory();
        canStop = true;
        lift.setPoseEstimate(new Pose2d(lift.getPoseEstimate().getX()+20, lift.getPoseEstimate().getY(), lift.getPoseEstimate().getHeading()));
        currentTrajectory = lift.trajectoryBuilder(lift.getPoseEstimate()).lineTo(new Vector2d(ground,0)).build();
        lift.followTrajectoryAsync(currentTrajectory);
        parallelo.down();
    }

    public void setPower(double power){
        if (lift.isBusy()) return; // nu interfera cu traiectorii
        lift.setPowerWithoutEncoders(liftSensor.isPressed()?Math.max(power, 0):power);
    }

    public void nextStack(){
        parallelo.down();
        if(stackPos == -1)
            stackPos = 4;
        currentTrajectory = lift.trajectoryBuilder(lift.getPoseEstimate()).lineTo(new Vector2d(3.25*stackPos + 0.5,0)).build();
        lift.followTrajectoryAsync(currentTrajectory);
        stackPos -=1;
    }

    public void update(){
        if(liftSensor.isPressed()){
            if (canStop) {
                lift.breakFollowing();
                canStop = false;
            }
            lift.setPoseEstimate(new Pose2d());
        }
        lift.update();
    }

    public Lift(HardwareMap hw){
        lift = new SampleLiftController(hw);
        parallelo = new Parallelo(hw);
        liftSensor = hw.get(TouchSensor.class, "senzoratingere");
        stackPos = 4;
    }
}
