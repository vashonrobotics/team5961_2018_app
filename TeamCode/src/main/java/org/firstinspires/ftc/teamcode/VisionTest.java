package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.internal.BlobDetector;
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;


/**
 * Created by FTC on 9/23/2017.
 */
@Autonomous(group = "5961", name = "Vision Test")
public class VisionTest extends LinearOpMode  {
    private ArrayList<DcMotor> baseMotorArrray = new ArrayList<>();
    double velX = 0;
    double velY = 0;

    @Override
    public void runOpMode() {
        baseMotorArrray.add(hardwareMap.dcMotor.get("motorLF"));
        baseMotorArrray.add(hardwareMap.dcMotor.get("motorRF"));
        baseMotorArrray.add(hardwareMap.dcMotor.get("motorLB"));
        baseMotorArrray.add(hardwareMap.dcMotor.get("motorRB"));
        baseMotorArrray.get(0).setDirection(DcMotorSimple.Direction.REVERSE);
        baseMotorArrray.get(1).setDirection(DcMotorSimple.Direction.REVERSE);
        // STUFF FOR BLOB DETECTION
        BlobDetector blobDetector = new BlobDetector();
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
            for (int i = 0; i < 5; i++){
                double[] ballPos = blobDetector.getBestCandidatePosition();
                ballPositions.add(ballPos);
                if (ballPos[2] > 10000 && ballPos[2] > 0.1) {
                    numTooClose++;
                }
                if (ballPos[2]<0.1){
                    numNotFound++;
                }
            }
            double  ballX = 0;
            double ballY = 0;
            for (double[] ballInfo: ballPositions){
                ballX += ballInfo[0];
                ballY += ballInfo[1];
            }
            ballX = ballX/(ballPositions.size()-numNotFound);
            ballY = ballY/(ballPositions.size()-numNotFound);
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
                    telemetry.addData("all ball poses: ", ballPos[1]);
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

            DriveTrain.mecanum(baseMotorArrray,velX*2, velY*2,0, false);
            sleep(100);
            DriveTrain.mecanum(baseMotorArrray,0, 0,0, false);

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
}
