package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.internal.BlobDetector;
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.opencv.core.Scalar;

import java.util.ArrayList;


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
        BlobDetector blobDetector = new BlobDetector(new Scalar(9, 100,50), new Scalar(38, 255,255), new Scalar(0,0,70), new Scalar(180, 60, 160), 10);
        ArrayList<double[]> previousBallPositions = new ArrayList<>();
        while (true) {
//            for (int i = 0; i < 10; i++) {
            FtcRobotControllerActivity.shouldProcessImage = true;
            while (FtcRobotControllerActivity.shouldProcessImage) { // should process image is turned to false after the image is processed
                sleep(5);
            }
//                telemetry.addLine("started processing");
//                telemetry.update();
            ArrayList<double[]> ballPositions = new ArrayList<>();
            int numTooClose = 0;
            int numNotFound = 0;
            double ballX = 0;
            double ballY = 0;
            double ballSize = 0;
            for (int i = 0; i < NUM_FRAMES_CONSIDERED; i++){
                double[] ballInfo = blobDetector.getBestCandidateData();
                ballX += ballInfo[0];
                ballY += ballInfo[1];
                ballSize += ballInfo[2];
                if (ballInfo[2] > 10000 && ballInfo[2] > 0.1) {
                    numTooClose++;
                }
                if (ballInfo[2]<0.1){
                    numNotFound++;
                }
            }
            ballX = ballX/(NUM_FRAMES_CONSIDERED-numNotFound);
            ballY = ballY/(NUM_FRAMES_CONSIDERED-numNotFound);
            ballSize = ballSize/(NUM_FRAMES_CONSIDERED-numNotFound);
            previousBallPositions.add(new double[]{ballX, ballY});
            telemetry.addData("position " + ballX + " ",ballY);
            telemetry.addData("horizontal power", -(ballX-288/2)*0.003);
            if (numTooClose < 3){// if ball is farther than 100cm move
                velY = (-(ballX-288/2)*0.003);
                if (velY > 0.2){
                    velY = 0.2;
                }
                    velX = 0.2;
                telemetry.addLine("go");

            }else {
                velX = 0;
                velY = 0;
                telemetry.addLine("too close");
                for (double[] ballPos: previousBallPositions) {
                    telemetry.addData("all ball Ys: ", ballPos[1]);
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

//            try {
////                for ()
//            }catch (IndexOutOfBoundsException){
//
//            }
//                telemetry.update();
//                for (int index = 0; index < blobDetector.getContourCenters().size(); index++){
//                    telemetry.addData("position is: " + blobDetector.getContourCenters().get(index)[0], blobDetector.getContourCenters().get(index)[1]);
////                    telemetry.addData("shape is: ", blobDetector.getContourTypes().get(index));
//                }
            if (makesSense(ballX, ballY, ballSize)){
                DriveTrain.mecanum(baseMotorArrray,velX*2, velY*2,0, false);
                sleep(100);
                DriveTrain.mecanum(baseMotorArrray,0, 0,0, false);

                if (posIdea.value[0] < 0 && posIdea.value[1] < 0) { // if first time
                   posIdea.trajectoryDirection = new double[]{0,0};
                   sizeIdea.trajectoryDirection = 0.0;
                } else {
                    sizeIdea.trajectoryDirection = ballSize - sizeIdea.value;
                    posIdea.trajectoryDirection = new double[]{ballX - posIdea.value[0], ballY - posIdea.value[1]};
                }
                posIdea.value = new double[] {ballX, ballY};
                sizeIdea.value = ballSize;
            }else{
                sleep(300);
            }


//            velY = 0;
            telemetry.addLine("done Processing");
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

    private boolean makesSense(double ballX, double ballY, double ballSize) {
        if (posIdea.value[0] < 0 && posIdea.value[1] < 0){// if it's the first frame just allow it
            return true;
        }
        double estimatedX = posIdea.value[0] + posIdea.trajectoryDirection[0];
        double estimatedY = posIdea.value[1] + posIdea.trajectoryDirection[1];

        double estimatedAngle = Math.tan(estimatedX/estimatedY);
        double actualAngle = Math.tan(ballX/ballY);
        double diffY = ballY - estimatedY;
        double diffX = ballX - estimatedX;

        double diffAngle = Math.tan(diffX/diffY);
        boolean sizeIsReasonable = sizeIdea.value < ballSize;
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
