package org.firstinspires.ftc.teamcode;

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

    private Rect[] zones;

    public CenterStagePropDetectionPipeline(Size targetRes, double[] targetColor) {
        this.targetColor = targetColor;
        this.targetRes = targetRes;
        this.input = new Mat(targetRes, CvType.CV_8UC4);
        this.zones = new Rect[]{
                new Rect(0, 0, (int) (0.1 * targetRes.width), (int) (0.1 * targetRes.height)),
                new Rect(1, 1, (int) (0.1 * targetRes.width), (int) (0.1 * targetRes.height)),
                new Rect(2, 2, (int) (0.1 * targetRes.width), (int) (0.1 * targetRes.height))
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
        Imgproc.resize(highresInput, input, input.size(), 0, 0, Imgproc.INTER_AREA);

        double minDistance = 1000;

        int i = 0;
        for (Rect zone : zones) {
            // get the submat of the input
            inputSubmat = input.submat(zone.y, zone.y + zone.height, zone.x, zone.x + zone.width);
            Scalar avgColor = Core.mean(inputSubmat);
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

        switch (viewportStage) {
            case 0: return highresInput;
            case 1: return input;
            case 2: return inputSubmat;
            case 3: viewportStage = 0;
                return highresInput;
        }

        return input;

    }



}
