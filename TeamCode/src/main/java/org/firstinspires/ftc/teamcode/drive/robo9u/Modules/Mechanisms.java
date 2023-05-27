package org.firstinspires.ftc.teamcode.drive.robo9u.Modules;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.easyopencv.OpenCvCamera;

@Config
public class Mechanisms {
    public Claw claw;
    public Lift lift;
    private Detection detection;

    public static double  minDistance = 85, maxDistance = 195;

    public void update(){
        claw.update();
        lift.update();
        if(lift.lift.isBusy() || lift.lift.getCurrentPosition() > 0)
            return;
    }

    public Mechanisms(HardwareMap hw)
    {
        claw = new Claw(hw);
        lift = new Lift(hw);
    }
}