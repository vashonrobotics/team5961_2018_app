package org.firstinspires.ftc.robotcontroller.internal;

import android.util.Log;

import com.sun.tools.javac.util.Pair;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.opencv.core.Core;
import org.opencv.core.CvException;
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
    Scalar minColorBorderRange;
    Scalar maxColorBorderRange;
    int borderThickness;
    public BlobDetector(Scalar minColorRange, Scalar maxColorRange, Scalar minColorBorderRange,
                        Scalar maxColorBorderRange, int borderThickness) {
        this.minColorRange = minColorRange;
        this.maxColorRange = maxColorRange;
        this.minColorBorderRange = minColorBorderRange;
        this.maxColorBorderRange = maxColorBorderRange;
        this.borderThickness = borderThickness;
    }
    public double[] getBestCandidateData() {
        ArrayList<MatOfPoint> contours = FtcRobotControllerActivity.imageData.fst;
        Mat hsvFrame = FtcRobotControllerActivity.imageData.snd;
        Core.rotate(hsvFrame, hsvFrame, Core.ROTATE_90_CLOCKWISE);

        Mat pixelsOfOneColor = new Mat();
        Mat pixelsOfBorder = new Mat();

        Imgproc.blur(hsvFrame, hsvFrame, new Size(10,10));
        Core.inRange(hsvFrame,minColorRange, maxColorRange, pixelsOfOneColor);
        Core.inRange(hsvFrame, minColorBorderRange, maxColorBorderRange, pixelsOfBorder);

//        Core.inRange(hsvFrame, new Scalar(90, 50, 50), new Scalar(150, 255, 255), blues);
        Mat kernal = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(2,2));
//        Imgproc.blur(pixelsOfOneColor, pixelsOfOneColor, new Size(10,10));
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
        ArrayList<double[]>contourData = new ArrayList<>();
        List<MatOfPoint> canidatesForBall = new ArrayList<>();
        for (int i = 0; i < contours.size(); i++) {
            MatOfPoint contour = contours.get(i);
            Rect rect = Imgproc.boundingRect(contour);
            Rect borderRect = new Rect(rect.x-borderThickness, rect.y-borderThickness,// upper left hand coords
                    rect.width+borderThickness*2, rect.height+borderThickness*2);
            int numIter = 0;
            while ((borderRect.x < 0 || borderRect.x + rect.width >= hsvFrame.cols()-1 ||
                    borderRect.y < 0 || borderRect.y + rect.height >= hsvFrame.rows()-1) && numIter < 100){
                borderThickness -= 1;
                borderRect = new Rect(rect.x-borderThickness, rect.y-borderThickness,// upper left hand coords
                        rect.width+borderThickness*2, rect.height+borderThickness*2);
                numIter++;
            }
            Mat contourFrame = hsvFrame.clone().submat(rect);
            Mat borderFrame = new Mat();
            borderFrame = hsvFrame.clone().submat(borderRect);
            double percentColor = 0;
            int numTimesIterated = 0;
            // find percent color of contour
            for (int row = 0; row < contourFrame.rows(); row++){
                for (int col=0; col < contourFrame.cols(); col++){
                    double[] pixel = contourFrame.get(row,col);
                    if (pixel[0] > minColorRange.val[0] && pixel[0] < maxColorRange.val[0] &&
                            pixel[1] > minColorRange.val[1] && pixel[1] < maxColorRange.val[1] &&
                            pixel[2] > minColorRange.val[2] && pixel[2] < maxColorRange.val[2]) {
                        percentColor += 1;
                    }
                    numTimesIterated++;
                }
            }
            percentColor = percentColor / numTimesIterated;
            int numTimesBorderIterated = 0;
            double percentBorderColor = 0;
            // find percent color of border
            for (int row = 0; row < borderFrame.rows(); row++){
                for (int col=0; col < borderFrame.cols(); col++) {
                    double[] pixel = borderFrame.get(row, col);
                    // if it's on the border
                    if ((row <= borderThickness || col <= borderThickness ||
                            row >= borderFrame.rows() - borderThickness || col >= borderFrame.cols() - borderThickness)) {
                        // if it's the right color
                        if (pixel[0] > minColorBorderRange.val[0] && pixel[0] < maxColorBorderRange.val[0] &&
                                pixel[1] > minColorBorderRange.val[1] && pixel[1] < maxColorBorderRange.val[1] &&
                                pixel[2] > minColorBorderRange.val[2] && pixel[2] < maxColorBorderRange.val[2]) {
                            percentBorderColor += 1;
                        }
                        numTimesBorderIterated++;
                    }
                }
            }
            if (numTimesBorderIterated > 0) {
                percentBorderColor = percentBorderColor / numTimesBorderIterated;
            }else {
                percentBorderColor = 1.0;
            }
            //Imgproc.contourArea(contour) > 40 && percentColor > 0.5
            // ~33500 pixels at ~50cm
            // ~9000 pixels at ~100cm
            // 5000 pixels at 150cm
            //
            double xPos = rect.y+rect.height/2; // because rotated
            double yPos = rect.x+rect.width/2;
            if  (percentColor > 0.5 && yPos < hsvFrame.rows()/3 && percentBorderColor > 0.5){
                contourData.add(new double[]{xPos, yPos, rect.height*rect.width});
                canidatesForBall.add(contour);
            }
        }
        try {
            return contourData.get(contourData.size() - 1);
        } catch (IndexOutOfBoundsException e){
            Log.d("", "Didn't find any candidates");
            return new double[]{0.0,0.0,0.0};
        }
    }

    private boolean isAboutEqual(double lhs, double rhs, double leniency){
        return lhs + leniency/2 > rhs && lhs - leniency/2 < rhs;
    }
}
