package org.firstinspires.ftc.teamcode.drive.robo9u.util;

import com.acmerobotics.dashboard.config.Config;

import org.checkerframework.checker.units.qual.C;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.imgproc.Imgproc;

@Config
public class SleevePipeline extends OpenCvPipeline {

    private int sleeveIndex;

    public static Point SLEEVE_TOPLEFT_ANCHOR_POINT = new Point(190, 10);
    public static int REGION_WIDTH = 80;
    public static int REGION_HEIGHT = 40;

    private final Scalar
            YELLOW  = new Scalar(255, 255, 0),
            CYAN    = new Scalar(0, 255, 255),
            MAGENTA = new Scalar(255, 0, 255);

    // Anchor point definitions
    Point sleeve_pointA = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y);
    Point sleeve_pointB = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    @Override
    public Mat processFrame(Mat input) {
        Mat areaMat = input.submat(new Rect(sleeve_pointA, sleeve_pointB));
        Scalar sumColors = Core.sumElems(areaMat);
        double minColor = Math.min(sumColors.val[0], Math.min(sumColors.val[1], sumColors.val[2]));
        if (sumColors.val[0] == minColor) {//red is minimum
            sleeveIndex = 2;
            Imgproc.rectangle(input, sleeve_pointA, sleeve_pointB, CYAN, 2);
        } else if (sumColors.val[1] == minColor) {//blue is minimum
            sleeveIndex = 1;
            Imgproc.rectangle(input, sleeve_pointA, sleeve_pointB, MAGENTA, 2);
        } else {//yellow is minimum
            sleeveIndex = 3;
            Imgproc.rectangle(input, sleeve_pointA, sleeve_pointB, YELLOW, 2);
        }
        areaMat.release();
        return input;
    }
    public int getSleeveIndex(){
        return sleeveIndex;
    }

}
