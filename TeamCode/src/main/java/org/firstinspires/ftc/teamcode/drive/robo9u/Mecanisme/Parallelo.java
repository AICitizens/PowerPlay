package org.firstinspires.ftc.teamcode.drive.robo9u.Mecanisme;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Parallelo {
    public static double up = 0.805;
    public static double down = 0.303;
    public Servo paralelo;
    Parallelo (HardwareMap hw){
        paralelo=hw.get(Servo.class,"paralello") ;
    }
    public void up(){
        paralelo.setPosition(up);
    }
    public void down(){
        paralelo.setPosition(down);
    }
}
