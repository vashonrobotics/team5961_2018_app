package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.internal.BlobDetector;
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.opencv.core.Scalar;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;


/**
 * Created by FTC on 9/23/2017.
 */
@Autonomous(group = "5961", name = "Vision Test")
public class VisionTest extends LinearOpMode  {
    private ArrayList<DcMotor> baseMotorArrray = new ArrayList<>();
    private double velX = 0;
    private double velY = 0;
    private Idea<double[]> posIdea = new Idea<>(new double[]{-1,-1}, new double[]{-1,-1});
    private Idea<Double> sizeIdea = new Idea<>(-1.0, -1.0);
    private int NUM_FRAMES_CONSIDERED = 5;

    @Override
    public void runOpMode() {
        baseMotorArrray.add(hardwareMap.dcMotor.get("motorLF"));
        baseMotorArrray.add(hardwareMap.dcMotor.get("motorRF"));
        baseMotorArrray.add(hardwareMap.dcMotor.get("motorLB"));
        baseMotorArrray.add(hardwareMap.dcMotor.get("motorRB"));
        baseMotorArrray.get(0).setDirection(DcMotorSimple.Direction.REVERSE);
        baseMotorArrray.get(1).setDirection(DcMotorSimple.Direction.REVERSE);
        // STUFF FOR BLOB DETECTION
        BlobDetector goldDetector = new BlobDetector(new Scalar(9, 100,50), new Scalar(38, 255,255), new Scalar(0,0,70), new Scalar(180, 60, 160), 5);
        BlobDetector silverDetector = new BlobDetector(new Scalar(0, 0,190), new Scalar(180, 25,255), new Scalar(0,0,70), new Scalar(180, 60, 160), 5);
        ArrayList<double[]> previousGoldPositions = new ArrayList<>();
        while (opModeIsActive()) {
//            for (int i = 0; i < 10; i++) {
            FtcRobotControllerActivity.shouldProcessImage = true;
            while (FtcRobotControllerActivity.shouldProcessImage) { // should process image is turned to false after the image is processed
                sleep(5);
            }
//                telemetry.addLine("started processing");
//                telemetry.update();
            ArrayList<double[]> possibleSilversData = new ArrayList<>();
            int numTooClose = 0;
            int numNotFound = 0;
            ArrayList<Double> goldXs = new ArrayList<>();
            ArrayList<Double> goldYs = new ArrayList<>();
            ArrayList<Double> goldSizes = new ArrayList<>();
            for (int i = 0; i < NUM_FRAMES_CONSIDERED; i++){
                ArrayList<double[]> goldDetectorCandidatesData = goldDetector.getCandidatesData();
                double[] goldInfo = goldDetectorCandidatesData.get(goldDetectorCandidatesData.size()-1);
                goldXs.add(goldInfo[0]);
                goldYs.add(goldInfo[1]);
                goldSizes.add(goldInfo[2]);
                possibleSilversData.addAll(silverDetector.getCandidatesData());
                if (goldInfo[2] > 10000 && goldInfo[2] > 0.1) {
                    numTooClose++;
                }
                if (goldInfo[2]<0.1){
                    numNotFound++;
                }
            }
            Collections.sort(goldXs);
            Collections.sort(goldYs);
            Collections.sort(goldSizes);
            double goldX = (goldXs.get((int)((goldXs.size()-1)/2)) + goldXs.get((int)(goldXs.size()/2)))/2;
            double goldY = (goldYs.get((int)((goldYs.size()-1)/2)) + goldYs.get((int)(goldYs.size()/2)))/2;
            double goldSize = (goldSizes.get((int)((goldSizes.size()-1)/2)) + goldSizes.get((int)(goldSizes.size()/2)))/2;
            Collections.sort(possibleSilversData, new Comparator<double[]>() {
                @Override
                public int compare(double[] lhs, double[] rhs) {
                    return Double.compare(lhs[2],rhs[2]);
                }
            });

            ArrayList<double[]> arrayOfSilverData = new ArrayList<>();
            int index = 0;
            double[] previousPossibleSilver = new double[]{};
            for (double[] possibleSilver:possibleSilversData){
                if (index > 0){
                    if (possibleSilver[2] - previousPossibleSilver[2] < 100) {
                        arrayOfSilverData.add(possibleSilver);
                        arrayOfSilverData.add(previousPossibleSilver);
                    }
                }
                index++;
                previousPossibleSilver = possibleSilver;
            }
//            goldX = goldX/(NUM_FRAMES_CONSIDERED-numNotFound);
//            goldY = goldY/(NUM_FRAMES_CONSIDERED-numNotFound);
//            goldSize = goldSize/(NUM_FRAMES_CONSIDERED-numNotFound);
            previousGoldPositions.add(new double[]{goldX, goldY});
            telemetry.addData("position " + goldX + " ",goldY);
            telemetry.addData("horizontal power", -(goldX-288/2)*0.003);
            if (numTooClose < 3){// if gold is farther than 100cm move
                velY = (-(goldX-288/2)*0.003);
                if (velY > 0.2){
                    velY = 0.2;
                }
                    velX = 0.2;
                telemetry.addLine("go");

            }else {
                velX = 0;
                velY = 0;
                telemetry.addLine("too close");
                for (double[] goldPos: previousGoldPositions) {
                    telemetry.addData("all gold Ys: ", goldPos[1]);
                }
                telemetry.update();
                sleep(5000);
            }
            if (numNotFound > 3){
                velX = 0;
                velY = 0;
                DriveTrain.mecanum(baseMotorArrray, 0,0,0, false);
                telemetry.addLine("stop");
                sleep(500);
            }
            if (makesSense(goldX, goldY, goldSize)){
                DriveTrain.mecanum(baseMotorArrray,velX*2, velY*2,0, true);
                sleep(200);
                DriveTrain.mecanum(baseMotorArrray,0, 0,0, false);

                if (posIdea.value[0] < 0 && posIdea.value[1] < 0) { // if first time
                   posIdea.trajectoryDirection = new double[]{0,0};
                   sizeIdea.trajectoryDirection = 0.0;
                } else {
                    sizeIdea.trajectoryDirection = goldSize - sizeIdea.value;
                    posIdea.trajectoryDirection = new double[]{goldX - posIdea.value[0], goldY - posIdea.value[1]};
                }
                posIdea.value = new double[] {goldX, goldY};
                sizeIdea.value = goldSize;
            }else{
                sleep(300);
            }


//            velY = 0;
            telemetry.update();
//            sleep(100);
        }
        // STUFF FOR KEYPOINT DETECTION
//        while (true) {
//        double min = 0;
//        double std = 0;
//        double std2 = 0;
//        double std3 = 0;
//        double med = 0;
//        double mean = 0;
//            for (int i = 0; i < 10; i++) {
//                FtcRobotControllerActivity.shouldProcessImage = true;
//                while (FtcRobotControllerActivity.shouldProcessImage) { // should process image is turned to false after the image is processed
//                    sleep(5);
//                }
//                if (FtcRobotControllerActivity.medianDist > 0) {
//                    min += FtcRobotControllerActivity.minDist < 180 ? 1:0;
//                    std += FtcRobotControllerActivity.minDist < 90 ? 1:0;
//                    std2 += FtcRobotControllerActivity.minDist < 270 ? 1:0;
//                    if (!Double.isNaN(FtcRobotControllerActivity.maxDist)) {
////                        std += FtcRobotControllerActivity.maxDist < 20 ? 1:0;
////                        std2 += FtcRobotControllerActivity.maxDist < 40 ? 1:0;
//                        std3 += FtcRobotControllerActivity.maxDist < 50 ? 1:0;
//                    }
//
//                    med += FtcRobotControllerActivity.medianDist;
//                    mean += FtcRobotControllerActivity.avgDist;
//
//                }
////                sleep(30);
//            }
//            telemetry.addData("image is recognized: ", min / med >= 0.5);
//            telemetry.addData("min", min/med);
//            telemetry.addData("minv2", std/med);
//            telemetry.addData("minv3", std2/med);
//            telemetry.addData("std3", std3/med);
//            telemetry.addData("med", med);
//            telemetry.addData("mean", mean/med);
//            telemetry.update();
////            sleep(5000);
//        }
    }

    private boolean makesSense(double goldX, double goldY, double goldSize) {
        if (posIdea.value[0] < 0 && posIdea.value[1] < 0){// if it's the first frame just allow it
            return true;
        }
        double estimatedX = posIdea.value[0] + posIdea.trajectoryDirection[0];
        double estimatedY = posIdea.value[1] + posIdea.trajectoryDirection[1];

        double estimatedAngle = Math.tan(estimatedX/estimatedY);
        double actualAngle = Math.tan(goldX/goldY);
        double diffY = goldY - estimatedY;
        double diffX = goldX - estimatedX;

        double diffAngle = Math.tan(diffX/diffY);
        boolean sizeIsReasonable = sizeIdea.value < goldSize;
        if (sizeIdea.trajectoryDirection > 0){
            sizeIsReasonable = !sizeIsReasonable;
        } else if (sizeIdea.trajectoryDirection > -0.1 && sizeIdea.trajectoryDirection < 0.-1){
            sizeIsReasonable = false; // needs to be true if it is an and below
        }
        if (Math.abs(diffY) < 30 || sizeIsReasonable){ // maybe should be and
            return true;
        }else {
            return false;
        }
    }
}
