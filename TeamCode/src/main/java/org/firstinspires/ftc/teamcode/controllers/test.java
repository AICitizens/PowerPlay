package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class test extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        liftController lift = new liftController(hardwareMap);
        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            lift.update();
        }
    }
}
