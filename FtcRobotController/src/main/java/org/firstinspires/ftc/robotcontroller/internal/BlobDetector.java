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
    Scalar minColorRange;
    Scalar maxColorRange;
    public BlobDetector(Scalar minColorRange, Scalar maxColorRange) {
        this.minColorRange = minColorRange;
        this.maxColorRange = maxColorRange;
    }
    public double[] getBestCandidatePosition() {
        ArrayList<MatOfPoint> contours = FtcRobotControllerActivity.imageData.fst;
        Mat hsvFrame = FtcRobotControllerActivity.imageData.snd;
        Core.rotate(hsvFrame, hsvFrame, Core.ROTATE_90_CLOCKWISE);

        Mat pixelsOfOneColor = new Mat();


        Core.inRange(hsvFrame,minColorRange, maxColorRange, pixelsOfOneColor);
//        Core.inRange(hsvFrame, new Scalar(90, 50, 50), new Scalar(150, 255, 255), blues);
        Mat kernal = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(2,2));
        Imgproc.blur(pixelsOfOneColor, pixelsOfOneColor, new Size(10,10));
        Imgproc.erode(pixelsOfOneColor, pixelsOfOneColor, kernal);
        Imgproc.dilate(pixelsOfOneColor,pixelsOfOneColor,kernal);
        Imgproc.findContours(pixelsOfOneColor, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

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
            double percentColor = 0;
            int numTimesIterated = 0;
            for (int row = 0; row < contourFrame.rows(); row++){
                for (int col=0; col < contourFrame.cols(); col++){
                    double[] pixel = contourFrame.get(row,col);
                    percentColor += pixel[0] > minColorRange.val[0] && pixel[0] < maxColorRange.val[0] ? 1:0;
                    numTimesIterated++;
                }
            }
            percentColor = percentColor / numTimesIterated;
            //Imgproc.contourArea(contour) > 40 && percentColor > 0.5
            // ~33500 pixels at ~50cm
            // ~9000 pixels at ~100cm
            // 5000 pixels at 150cm
            //
            double xPos = rect.y+rect.height/2; // because rotated
            double yPos = rect.x+rect.width/2;
            if  (percentColor > 0.5 && yPos < hsvFrame.rows()/2){
                contourCenters.add(new double[]{xPos, yPos, rect.height*rect.width});
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
