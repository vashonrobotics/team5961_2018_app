package org.firstinspires.ftc.teamcode;

import android.util.Log;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfInt4;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class BlobDetector {
    private Scalar minColorRange;
    private Scalar maxColorRange;
    private Scalar minColorBorderRange;
    private Scalar maxColorBorderRange;
    public ArrayList<Double> aspectRatios = new ArrayList<>();
    private int borderThickness;
    public BlobDetector(Scalar minColorRange, Scalar maxColorRange) {
        this.minColorRange = minColorRange;
        this.maxColorRange = maxColorRange;
    }
    public ArrayList<BlobDetectorCandidate> getCandidatesData() {
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Mat hsvFrame = FtcRobotControllerActivity.imageData.snd;
        Mat pixelsOfOneColor = new Mat();

        Imgproc.blur(hsvFrame, hsvFrame, new Size(10,10));
        Core.inRange(hsvFrame,minColorRange, maxColorRange, pixelsOfOneColor);
//        Core.inRange(hsvFrame, minColorBorderRange, maxColorBorderRange, pixelsOfBorder);

//        Core.inRange(hsvFrame, new Scalar(90, 50, 50), new Scalar(150, 255, 255), blues);
        Mat kernal = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(2,2));
//        Imgproc.blur(pixelsOfOneColor, pixelsOfOneColor, new Size(10,10));
        Imgproc.erode(pixelsOfOneColor, pixelsOfOneColor, kernal);
        Imgproc.dilate(pixelsOfOneColor,pixelsOfOneColor,kernal);
        for (int col = 0; col < pixelsOfOneColor.cols();col++){
            for (int row = 0; row < pixelsOfOneColor.rows();row++){
                if (pixelsOfOneColor.get(row,col)[0] < 100) {
                    pixelsOfOneColor.put(row,col, 0.0);
                }
            }
        }
//        double pixel = 0;
//        if (pixel < 100){
//            pixel = 0;
//        }
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
        ArrayList<BlobDetectorCandidate>contourData = new ArrayList<>();
        List<MatOfPoint> canidatesForBall = new ArrayList<>();
        for (int i = 0; i < contours.size(); i++) {
            MatOfPoint contour = contours.get(i);
            Rect rect = Imgproc.boundingRect(contour);
            Mat contourFrame = hsvFrame.clone().submat(rect);

            double percentColor = 0;
            int numTimesIterated = 0;
            // find percent color of contour
            for (int row = 0; row < contourFrame.rows(); row++){
                for (int col=0; col < contourFrame.cols(); col++){
                    double[] pixel = contourFrame.get(row,col);
                    if (pixel[0] > minColorRange.val[0]&& pixel[0] < maxColorRange.val[0]){// &&
                            //pixel[1] > minColorRange.val[1] && pixel[1] < maxColorRange.val[1] &&
                           // pixel[2] > minColorRange.val[2] && pixel[2] < maxColorRange.val[2]) {
                        percentColor += 1;
                    }
                    numTimesIterated++;
                }
            }
            percentColor = percentColor / numTimesIterated;
            //Imgproc.contourArea(contour) > 40 && percentColor > 0.5
            // ~33500 pixels at ~50cm
            // ~9000 pixels at ~100cm
            // 5000 pixels at 150cm
            //
            double xPos = rect.x+rect.width/2; // because rotated
            double yPos = hsvFrame.rows() - (rect.y+rect.height/2);
            double MIN_ASPECT_RATIO = 0.6;
            MatOfInt hull = new MatOfInt();
//            Imgproc.convexHull(contour, hull);
//            MatOfInt4 convexityDefects = new MatOfInt4();

//            Imgproc.convexityDefects(contour, hull, convexityDefects); // could use convexity defects to test for convexity
//            Log.d("PercentColor ",String.valueOf(percentColor));
//            Log.d("PercentBorderColor ",String.valueOf(percentBorderColor));//yPos < hsvFrame.rows()/3
            // width = cols = 288 height = rows = 384
            if  (percentColor > 0.6 && yPos < hsvFrame.rows()*2/3){// && percentBorderColor > 0.5 && yPos < hsvFrame.rows()*2/3){
//                    && rect.width/rect.height > MIN_ASPECT_RATIO && rect.height/rect.width > MIN_ASPECT_RATIO){
                contourData.add(new BlobDetectorCandidate(xPos, yPos, rect.width, rect.height));
                canidatesForBall.add(contour);
                aspectRatios.add((double) percentColor);
            }
        }
        try {
            return contourData;
        } catch (IndexOutOfBoundsException e){
            Log.d("", "Didn't find any candidates");
            return contourData;
        }
    }

    private boolean isAboutEqual(double lhs, double rhs, double leniency){
        return lhs + leniency/2 > rhs && lhs - leniency/2 < rhs;
    }
}
