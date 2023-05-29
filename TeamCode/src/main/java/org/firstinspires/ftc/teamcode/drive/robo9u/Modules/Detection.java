package org.firstinspires.ftc.teamcode.drive.robo9u.Modules;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.robo9u.util.SleevePipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class Detection {

    private OpenCvCamera camera;

    private SleevePipeline sleevePipeline;

    public Detection(HardwareMap hardwareMap, String webcamName){
        sleevePipeline = new SleevePipeline();
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
        setSleeveDetectionMode();
    }

    public void setSleeveDetectionMode(){
        camera.setPipeline(sleevePipeline);
    }
    public int getParkingIndex(){
        return sleevePipeline.getSleeveIndex();
    }
    public void stopCamera(){
        camera.stopStreaming();
    }
}
