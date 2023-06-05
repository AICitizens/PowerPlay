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
    public LiftState liftState = LiftState.Idle;

    public TouchSensor liftSensor;
    public LiftController lift;
    public FourBar fourBar;
    public static double ground = 0, low = 24, mid = 47 , high = 72, stackConeDist = 3.25, stackPos;

    private boolean manualControl = false;

    public void stopCurrentTrajectory(){
        lift.stop();
    }

    public void setLiftState(LiftState state){
        liftState = state;
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
        liftState = LiftState.Idle;
        fourBar.down();
        if(stackPos == -1)
            stackPos = 4;
        lift.setTarget(stackConeDist*stackPos + 0.5+ground);
        stackPos -=1;
    }

    public void update(){
        switch (liftState){
            case High:
                lift.setTarget(high+ground);
                fourBar.up();
                break;
            case Mid:
                lift.setTarget(mid+ground);
                fourBar.up();
                break;
            case Low:
                lift.setTarget(low+ground);
                fourBar.up();
                break;
            case Ground:
                lift.setTarget(ground);
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
        stackPos = 4;
    }
}
