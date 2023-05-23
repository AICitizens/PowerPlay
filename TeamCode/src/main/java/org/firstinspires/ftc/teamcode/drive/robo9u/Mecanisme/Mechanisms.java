package org.firstinspires.ftc.teamcode.drive.robo9u.Mecanisme;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
public class Mechanisms {
    public Claw claw;
    public Lift lift;
    private Rev2mDistanceSensor distanceSensor = null;

    private FtcDashboard dashboard = null;

    private OpenCvCamera webcam;

    public static double  minDistance = 85, maxDistance = 195;

    public void update(){
        claw.update();
        dashboard.getTelemetry().addData("distance", distanceSensor.getDistance(DistanceUnit.MM));
        lift.update();
        dashboard.getTelemetry().update();
        if(lift.lift.isBusy() || !lift.lift.getPoseEstimate().equals(new Pose2d()))
            return;
        if(minDistance <= distanceSensor.getDistance(DistanceUnit.MM) && distanceSensor.getDistance(DistanceUnit.MM) <= maxDistance)
            claw.Close();
    }

    public Mechanisms(HardwareMap hw)
    {
        claw = new Claw(hw);
        lift = new Lift(hw);

        dashboard = FtcDashboard.getInstance();

        distanceSensor = hw.get(Rev2mDistanceSensor.class, "distanceSensor");

    }
}