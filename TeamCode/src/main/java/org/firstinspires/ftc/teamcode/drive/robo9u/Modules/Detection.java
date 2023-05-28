package org.firstinspires.ftc.teamcode.drive.robo9u.Modules;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.robo9u.util.ConeAndJunctionPipeline;
import org.firstinspires.ftc.teamcode.drive.robo9u.util.SleevePipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class Detection {

    private OpenCvCamera camera;

    private SleevePipeline sleevePipeline;
    private ConeAndJunctionPipeline coneAndJunctionPipeline;

    public Detection(HardwareMap hardwareMap, String webcamName){
        sleevePipeline = new SleevePipeline();
        coneAndJunctionPipeline = new ConeAndJunctionPipeline();
        setup(hardwareMap, webcamName);
    }

    public void setup(HardwareMap hardwareMap, String webcamName){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        camera.openCameraDeviceAsync( new OpenCvCamera.AsyncCameraOpenListener( ) {
            @Override
            public void onOpened( ) {
                camera.startStreaming( 320,240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError( int errorCode ) {
            }
        });
        setConeAndJunctionDetectionMode();
    }

    public void setSleeveDetectionMode(){
        camera.setPipeline(sleevePipeline);
    }
    public void setConeAndJunctionDetectionMode(){
        camera.setPipeline(coneAndJunctionPipeline);
    }
    public int getParkingIndex(){
        return sleevePipeline.getSleeveIndex();
    }
    public boolean coneDetected(){
        return coneAndJunctionPipeline.coneDetected();
    }
    public boolean junctionDetected(){
        return coneAndJunctionPipeline.junctionDetected();
    }
    public void stopCamera(){
        camera.stopStreaming();
    }
}
