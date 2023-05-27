package org.firstinspires.ftc.teamcode.drive.robo9u.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.drive.LiftController;

@Config
public class Lift {
    TouchSensor liftSensor;
    public LiftController lift;
    public FourBar fourBar;
    public boolean canStop = false;
    public static double low = 9, mid = 32, high = 58, stackConeDist = 3.25, stackPos;

    public void stopCurrentTrajectory(){
        lift.stop();
        canStop = false;
    }

    public void goToHigh(){
        stopCurrentTrajectory();
        canStop = false;
        lift.setTarget(high);
        fourBar.up();
    }

    public void goToMid(){
        stopCurrentTrajectory();
        canStop = false;
        lift.setTarget(mid);
        fourBar.up();
    }

    public void goToLow(){
        stopCurrentTrajectory();
        canStop = false;
        lift.setTarget(low);
        fourBar.up();
    }

    public void retractFully(){
        stopCurrentTrajectory();
        canStop = true;
        lift.setTarget(0);
        fourBar.down();
    }

    public void setPower(double power){
        if (lift.isBusy()) return; // nu interfera cu traiectorii
        lift.setPower(liftSensor.isPressed()?Math.max(power, 0):power);
    }

    public void nextStack(){
        fourBar.down();
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
        }
        lift.update();
    }

    public Lift(HardwareMap hw){
        lift = new LiftController(hw);
        fourBar = new FourBar(hw);
        liftSensor = hw.get(TouchSensor.class, "senzoratingere");
        stackPos = 4;
    }
}
