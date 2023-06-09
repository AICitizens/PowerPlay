package org.firstinspires.ftc.teamcode.drive.robo9u.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.LiftController;

@Config
public class Lift {
    public enum LiftState{
        High,
        Mid,
        Low,
        Ground,
        STACK,
        Idle,
    }
    public LiftState liftState = LiftState.Idle;

    private ElapsedTime fourbarTimer;

    public TouchSensor liftSensor;
    public LiftController lift;
    public FourBar fourBar;
    public static double ground = 0, low = 23, mid = 46 , high = 71, stackConeDist = 3.25, stackPos;

    private boolean manualControl = false;

    public void stopCurrentTrajectory(){
        lift.stop();
    }

    public void setLiftState(LiftState state){
        liftState = state;
        fourbarTimer.reset();
    }

    public void setPower(double power){
        manualControl = false;
        if (lift.isBusy()) return; // nu interfera cu traiectorii
        if(power==0) return;
        liftState = LiftState.Idle;
        manualControl = true;
        lift.setPower(liftSensor.isPressed()?Math.max(power, 0):power);
        lift.stop();
    }

    public void nextStack(){
        if(stackPos == 0)
            stackPos = 5;
        stackPos -=1;
        liftState = LiftState.STACK;
    }

    public void update(){
        switch (liftState){
            case High:
                lift.setTarget(high+ground);
                if(fourbarTimer.milliseconds()>=125)
                    fourBar.up();
                break;
            case Mid:
                lift.setTarget(mid+ground);
                if(fourbarTimer.milliseconds()>=125)
                    fourBar.up();
                break;
            case Low:
                lift.setTarget(low+ground);
                if(fourbarTimer.milliseconds()>=125)
                    fourBar.up();
                break;
            case Ground:
                lift.setTarget(ground);
                fourBar.down();
                break;
            case STACK:
                lift.setTarget(stackConeDist*stackPos + 0.2+ground);
                fourBar.down();
                break;
            case Idle:
                break;
        }
        if(liftSensor.isPressed()){
                if(liftState == LiftState.Idle)
                    lift.stop();
                ground = LiftController.encoderTicksToCM(lift.getCurrentPosition());
        }
        if(!manualControl)
            lift.update();
    }

    public Lift(HardwareMap hw){
        lift = new LiftController(hw, true);
        fourBar = new FourBar(hw);
        liftSensor = hw.get(TouchSensor.class, "senzoratingere");
        stackPos = 5;
        fourbarTimer = new ElapsedTime();
        fourbarTimer.reset();
    }
}
