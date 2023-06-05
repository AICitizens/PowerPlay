package org.firstinspires.ftc.teamcode.drive.robo9u.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class FourBar {
    public static double up = 0.5;
    public static double down = 0.26;
    public Servo servo;
    FourBar(HardwareMap hw){
        servo=hw.get(Servo.class,"paralello") ;
    }
    public void up(){
            servo.setPosition(up);
    }
    public void down(){
        servo.setPosition(down);
    }
}
