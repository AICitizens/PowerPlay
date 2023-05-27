package org.firstinspires.ftc.teamcode.drive.robo9u.Mecanisme;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.controllers.LiftController;

@Config
public class Lift {
    TouchSensor liftSensor;
    public LiftController lift;
    public Parallelo parallelo;
    public boolean canStop = false;
    public static double low = 9, mid = 32, high = 58, stackConeDist = 3.25, stackPos;

    Trajectory currentTrajectory;

    public void stopCurrentTrajectory(){
        lift.stop();
        canStop = false;
    }

    public void goToHigh(){
        stopCurrentTrajectory();
        canStop = false;
        lift.setTarget(high);
        parallelo.up();
    }

    public void goToMid(){
        stopCurrentTrajectory();
        canStop = false;
        lift.setTarget(mid);
        parallelo.up();
    }

    public void goToLow(){
        stopCurrentTrajectory();
        canStop = false;
        lift.setTarget(low);
        parallelo.up();
    }

    public void retractFully(){
        stopCurrentTrajectory();
        canStop = true;
        lift.setTarget(0);
        parallelo.down();
    }

    public void setPower(double power){
        if (lift.isBusy()) return; // nu interfera cu traiectorii
        lift.setPower(liftSensor.isPressed()?Math.max(power, 0):power);
    }

    public void nextStack(){
        parallelo.down();
        if(stackPos == -1)
            stackPos = 4;
        lift.setTarget(stackConeDist*stackPos + 0.5);
        stackPos -=1;
    }

    public void update(){
        if(liftSensor.isPressed()){
            if (canStop) {
                lift.stop();
                canStop = false;
            }
            lift.stopAndResetEncoders();
        }
        lift.update();
    }

    public Lift(HardwareMap hw){
        lift = new LiftController(hw);
        parallelo = new Parallelo(hw);
        liftSensor = hw.get(TouchSensor.class, "senzoratingere");
        stackPos = 4;
    }
}
