package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.MoreMath.clamp;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class CenterStagePropDetectionPipeline extends OpenCvPipeline {

    private Mat input;
    private Mat inputSubmat;
    public int viewportStage = 0;
    private Size targetRes;
    public double[] targetColor;
    public boolean debug = true;

    private int minDistanceHaver = -1;
    private double zoneWidening = 30;

    private Rect[] zones;
    public double minDistance;
    public Scalar avgColor1;

    public static Size processingSize = new Size(1280, 720);
    public static Size testImgSize = new Size(662, 371);
    public Rect getRectForBox(double x, double y, double w, double h) {
        double leftBound = clamp(x - zoneWidening/2, 0, testImgSize.width - 2);
        double rightBound = clamp(x + w + zoneWidening, 0, testImgSize.width - 2);
        return new Rect((int) (leftBound/testImgSize.width * processingSize.width),
                (int) (y/testImgSize.height * processingSize.height),
                (int) ((rightBound - leftBound)/testImgSize.width * processingSize.width),
                (int) (h/testImgSize.height * processingSize.height));
    }
    public CenterStagePropDetectionPipeline(Size targetRes, double[] targetColor) {
        this.targetColor = targetColor;
        this.targetRes = targetRes;
        this.input = new Mat(processingSize, CvType.CV_8UC4);
        int sizeX = (int) (120.0/496.0 * 1280.0);
        int sizeY = (int) (125.0/496.0 * 720.0);
        // 365 169
        this.zones = new Rect[]{
                getRectForBox(172, 207, 91, 89),
                getRectForBox(363, 194, 80, 76),
                getRectForBox(555, 206, 90, 90)
        };
    }

    @Override
    public void onViewportTapped()
    {
        viewportStage += 1;
    }

    public void setTargetColor(double[] color) {
        this.targetColor = color;
    }

    public int getZone() {
        return minDistanceHaver;
    }
    @Override
    public Mat processFrame(Mat highresInput)
    {

        // downsample the input
        highresInput.copyTo(input);

        minDistance = 1000000;

        int i = 0;
        for (Rect zone : zones) {
            // get the submat of the input
            inputSubmat = input.submat(zone.y, zone.y + zone.height, zone.x, zone.x + zone.width);
            Scalar avgColor = Core.mean(inputSubmat);
            avgColor1 = avgColor;
            double distance = Math.sqrt(
                    Math.pow(targetColor[0] - avgColor.val[0], 2)
                            + Math.pow(targetColor[1] - avgColor.val[1], 2)
                            + Math.pow(targetColor[2] - avgColor.val[2], 2)
            );
            if (distance < minDistance) {
                minDistanceHaver = i;
                minDistance = distance;
            }
            i += 1;
        }

        i = 0;
        for (Rect zone : zones) {
            Imgproc.rectangle(input, new Rect(zone.x, zone.y, zone.width, zone.height),
                    (minDistanceHaver == i) ? new Scalar(0, 255, 0) : new Scalar(255, 0, 0));
            i += 1;
        }


        switch (viewportStage) {
            case 0: return highresInput;
            case 1: return input;
            case 2: viewportStage = 0;
                return highresInput;
        }

        return input;

    }



}
