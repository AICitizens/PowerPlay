package org.firstinspires.ftc.teamcode.drive.robo9u;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Drive {

    public SampleMecanumDrive drive;

    public Drive(HardwareMap hw)
    {
        drive = new SampleMecanumDrive(hw);
    }

}
