    package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.BlobDetector;
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

@Autonomous(group = "5961", name = "get object color")
public class GetColorOfObject extends LinearOpMode {
    @Override
    public void runOpMode() {
        FtcRobotControllerActivity.shouldProcessImage = true;
        while (FtcRobotControllerActivity.shouldProcessImage){
            sleep(5);
        }
        Mat hsvFrame = FtcRobotControllerActivity.imageData.snd;
        Mat white = new Mat();
        Core.inRange(hsvFrame, new Scalar(0,0,150), new Scalar(180, 60, 255), white);
        Mat kernal = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(2,2));
        Imgproc.blur(white, white, new Size(10,10));
        Imgproc.erode(white, white, kernal);
        Imgproc.dilate(white,white,kernal);
        Mat notWhite = new Mat();
        Core.bitwise_not(white, notWhite);
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(notWhite, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Collections.sort(contours, new Comparator<MatOfPoint>() {
                @Override
                public int compare(MatOfPoint lhs, MatOfPoint rhs) {
                    return Double.compare(Imgproc.contourArea(lhs), Imgproc.contourArea(rhs));
                }
            });

        Mat partOfImage = hsvFrame.submat(Imgproc.boundingRect(contours.get(contours.size()-1))); // get part of image of biggest contour
        double[] color = new double[]{0.0,0.0,0.0};
        double minH = 500;
        double minS = 500;
        double minV = 500;
        double maxH = 0;
        double maxS = 0;
        double maxV = 0;
        for (int row = 0; row < partOfImage.rows(); row++){
            for (int col = 0; col < partOfImage.cols(); col++) {
                double[] pixel = partOfImage.get(row,col);
                if (color[0] < minH && color[0] > 0.1) {
                    minH = color[0];
                }else if (color[0] > maxH) {
                    maxH = color[0];
                }
                if (color[1] < minS && color[1] > 0.1) {
                    minS = color[1];
                }else if (color[1] > maxS) {
                    maxS = color[1];
                }
                if (color[2] < minV && color[2] > 0.1) {
                    minV = color[2];
                }else if (color[2] > maxV) {
                    maxV = color[2];
                }
                color[0] += pixel[0];
                color[1] += pixel[1];
                color[2] += pixel[2];
            }
        }
        for (int i = 0; i < color.length;i++){
            color[i] = color[i] / (partOfImage.rows()*partOfImage.cols());
        }
        telemetry.addData("color: "+ color[0]
                + " " + color[1], color[2]);
        telemetry.addData("lowest vals: " + minH + " " + minS, minV);
        telemetry.addData("highest vals: " + maxH + " " + maxS, maxV);
        telemetry.update();
        sleep(1000000);
    }
}
