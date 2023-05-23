package org.firstinspires.ftc.teamcode.drive.robo9u.Mecanisme;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Claw {
    public static ElapsedTime elapsedTimer;
    public static boolean shouldClose = false;
    public static double closed = 0.57,  open = 0.36, timetoclose = 63;

    public Servo servo = null;

    Claw(HardwareMap hw){
        servo = hw.get(Servo.class, "Gripper");
        elapsedTimer = new ElapsedTime();
    }
    public void dropConeAndKeepBeacon() {
        if(elapsedTimer.milliseconds() >= 1000) {
            Open();
            elapsedTimer.reset();
            shouldClose = true;
        }
    }
    public void Close(){
        servo.setPosition(closed);
    }
    public void Open(){
        servo.setPosition(open);
    }
    public void update(){
        if(elapsedTimer.milliseconds() >= timetoclose && shouldClose){
            shouldClose = false;
            Close();
        }
    };
}
