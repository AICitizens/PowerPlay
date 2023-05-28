package org.firstinspires.ftc.teamcode.drive.robo9u.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Mechanisms {
    public Claw claw;
    public Lift lift;

    public void update(){
        claw.update();
        lift.update();
    }

    public Mechanisms(HardwareMap hw)
    {
        claw = new Claw(hw);
        lift = new Lift(hw);
    }
}