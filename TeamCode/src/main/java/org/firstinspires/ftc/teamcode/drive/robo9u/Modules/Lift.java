package org.firstinspires.ftc.teamcode.drive.robo9u.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.drive.LiftController;

@Config
public class Lift {
    public enum LiftState{
        High,
        Mid,
        Low,
        Ground,
        Idle,
    }
    LiftState liftState = LiftState.Idle;

    TouchSensor liftSensor;
    public LiftController lift;
    public FourBar fourBar;
    public static double ground = 0, low = 9, mid = 32, high = 58, stackConeDist = 3.25, stackPos;

    private boolean canStop = false;
    private boolean lowerInto = false;

    public void stopCurrentTrajectory(){
        lift.stop();
        canStop = false;
    }

    public void setLiftState(LiftState state){
        liftState = state;
    }

    public void lowerIntoJunction(boolean bool){
        lowerInto = bool;
    }

    public void setPower(double power){
        if (lift.isBusy()) return; // nu interfera cu traiectorii
        liftState = LiftState.Idle;
        lift.setPower(lift.getCurrentPosition()<=0?Math.max(power, 0):power);
    }

    public void nextStack(){
        fourBar.down();
        if(stackPos == -1)
            stackPos = 4;
        lift.setTarget(stackConeDist*stackPos + 0.5+ground);
        stackPos -=1;
    }

    public void update(){
        switch (liftState){
            case High:
                lift.setTarget(high+ground-(lowerInto?5:0));
                break;
            case Mid:
                lift.setTarget(mid+ground-(lowerInto?5:0));
                break;
            case Low:
                lift.setTarget(low+ground-(lowerInto?5:0));
                break;
            case Ground:
                lift.setTarget(ground);
                break;
            case Idle:
                break;
        }
        if(liftSensor.isPressed()){
            if (canStop) {
                lift.stop();
                canStop = false;
            }
        }
        if(liftState != LiftState.Idle) lift.update();
    }

    public Lift(HardwareMap hw){
        lift = new LiftController(hw);
        fourBar = new FourBar(hw);
        liftSensor = hw.get(TouchSensor.class, "senzoratingere");
        stackPos = 4;
    }
}
