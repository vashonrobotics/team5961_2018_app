package org.firstinspires.ftc.robotcontroller.internal;

import android.util.Log;

import com.sun.tools.javac.util.Pair;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class BlobDetector {
    public BlobDetector() {}
    public double[] getBestCandidatePosition() {
        ArrayList<MatOfPoint> contours = FtcRobotControllerActivity.imageData.fst;
        Mat hsvFrame = FtcRobotControllerActivity.imageData.snd;
        Core.rotate(hsvFrame, hsvFrame, Core.ROTATE_90_CLOCKWISE);

        Mat reds = new Mat();
        Mat blues = new Mat();

        Mat redsPart1 = new Mat();
        Mat redsPart2 = new Mat();
        Core.inRange(hsvFrame, new Scalar(170, 100, 100), new Scalar(180, 255, 255), redsPart1);
        Core.inRange(hsvFrame, new Scalar(1,100,100), new Scalar(7, 255, 255), redsPart2);
        Core.add(redsPart1, redsPart2, reds);
        Core.inRange(hsvFrame, new Scalar(90, 50, 50), new Scalar(150, 255, 255), blues);
        Mat kernal = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(2,2));
        Imgproc.blur(reds, reds, new Size(10,10));
        Imgproc.erode(reds, reds, kernal);
        Imgproc.dilate(reds,reds,kernal);
        Imgproc.findContours(reds, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        Collections.sort(contours, new Comparator<MatOfPoint>() {
            @Override
            public int compare(MatOfPoint lhs, MatOfPoint rhs) {
                Rect lhRect = Imgproc.boundingRect(lhs);
                Rect rhRect = Imgproc.boundingRect(rhs);
                return Double.compare(lhRect.height*lhRect.width, rhRect.height*rhRect.width);
            }
        });
        // look for ball
        ArrayList<double[]>contourCenters = new ArrayList<>();
        List<MatOfPoint> canidatesForBall = new ArrayList<>();
        for (int i = 0; i < contours.size(); i++) {
            MatOfPoint contour = contours.get(i);
            Rect rect = Imgproc.boundingRect(contour);
            Mat contourFrame = hsvFrame.clone().submat(rect);
            double percentRed = 0;
            int numTimesIterated = 0;
            for (int row = 0; row < contourFrame.rows(); row++){
                for (int col=0; col < contourFrame.cols(); col++){
                    double[] pixel = contourFrame.get(row,col);
                    percentRed += pixel[0] > 170 || pixel[0] < 7 ? 1:0;
                    numTimesIterated++;
                }
            }
            percentRed = percentRed / numTimesIterated;
            //Imgproc.contourArea(contour) > 40 && percentRed > 0.5
            // ~33500 pixels at ~50cm
            // ~9000 pixels at ~100cm
            // 5000 pixels at 150cm
            //
            if  (percentRed > 0.5){
                contourCenters.add(new double[]{rect.y+rect.height/2, rect.x+rect.width/2, rect.height*rect.width}); // rotation means that we need to switch x and y
                canidatesForBall.add(contour);
            }
        }
        try {
            return contourCenters.get(contourCenters.size() - 1);
        } catch (IndexOutOfBoundsException e){
            Log.d("", "Didn't find any candidates");
            return new double[]{0.0,0.0,0.0};
        }
    }

    private boolean isAboutEqual(double lhs, double rhs, double leniency){
        return lhs + leniency/2 > rhs && lhs - leniency/2 < rhs;
    }
}
