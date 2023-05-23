package org.firstinspires.ftc.teamcode.drive.robo9u.util.OpenCv;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.robo9u.util.OpenCv.DetectieCon;
import org.firstinspires.ftc.teamcode.drive.robo9u.util.OpenCv.hsv.pipeline_hsv;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class DetectieCon {

    private OpenCvCamera camera;
    private pipeline_hsv pipeline;

    public DetectieCon(HardwareMap hardwareMap, String webcamName){
        pipeline = new pipeline_hsv();
        setup(hardwareMap, webcamName);
    }

    public void setup(HardwareMap hardwareMap, String webcamName){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        camera.setPipeline(pipeline);
    }
    public void init(){
        openCameraDevice();
    }
    public void openCameraDevice( ) {

        camera.openCameraDeviceAsync( new OpenCvCamera.AsyncCameraOpenListener( ) {
            @Override
            public void onOpened( ) {
                camera.startStreaming( 320,240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError( int errorCode ) {
            }
        } );
    }
    public int getParkingIndex(){
        switch (pipeline.get_culoare()){
            case MAGENTA:
                return 0;
            case TURCOAZ:
                return 1;
            case GALBEN:
                return 2;
        }
        return 1;
    }
    public void stopCamera(){
        camera.stopStreaming();
    }
}
