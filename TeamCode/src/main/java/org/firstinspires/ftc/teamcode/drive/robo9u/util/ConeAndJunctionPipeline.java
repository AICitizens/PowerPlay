package org.firstinspires.ftc.teamcode.drive.robo9u.util;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class ConeAndJunctionPipeline extends OpenCvPipeline {
    public static Rect  ConePickupArea = new Rect(0, 100, 50, 50),
                        JunctionArea = new Rect(180, 0, 20, 100);

    public static Scalar    lowerYellow = new Scalar(20,70,80),
                            upperYellow = new Scalar(32, 255, 255);

    public static int junctionAverage = 0, blueConeAverage = 0, redConeAverage = 0;

    private boolean detectedCone = false;
    private boolean detectedJunction = false;

    public void checkJunction(Mat input){
        Mat optimizedInput = new Mat();
        Imgproc.cvtColor(input.submat(JunctionArea), optimizedInput, Imgproc.COLOR_RGB2HSV);
        Core.inRange(optimizedInput, lowerYellow, upperYellow, optimizedInput);
        detectedJunction = Core.sumElems(optimizedInput).val[0] > junctionAverage;
    }

    public void checkCone(Mat input){
        Mat optimizedInput = input.submat(ConePickupArea);
        detectedCone = Core.sumElems(optimizedInput).val[0] > redConeAverage || Core.sumElems(optimizedInput).val[2] > blueConeAverage;
    }

    @Override
    public Mat processFrame(Mat input) {
        checkJunction(input);
        checkCone(input);
        Imgproc.rectangle(input, ConePickupArea, detectedCone?upperYellow:new Scalar(0));
        Imgproc.rectangle(input, JunctionArea, detectedJunction?upperYellow:new Scalar(0));
        return input;
    }

    public boolean coneDetected(){
        return detectedCone;
    }
    public boolean junctionDetected(){
        return detectedJunction;
    }
}
