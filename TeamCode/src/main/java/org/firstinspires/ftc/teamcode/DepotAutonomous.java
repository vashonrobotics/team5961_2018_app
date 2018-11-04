package org.firstinspires.ftc.teamcode;

import android.graphics.Camera;
import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.BlobDetector;
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.android.JavaCameraView;
import org.opencv.core.Scalar;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;

@Autonomous(group = "5961", name = "Depot Autonomous")
public class DepotAutonomous extends LinearOpMode {
    private ArrayList baseMotorArray = new ArrayList();
    private DcMotor lift;
    private Servo markerDropper;
//    VuforiaLocalizer vuforia;
//    private final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;
//    private Boolean targetVisible = false;
//    private OpenGLMatrix lastLocation = null;
//    VuforiaTrackable blueRover;
//    VuforiaTrackable redFootprint;
//    VuforiaTrackable frontCraters;
//    VuforiaTrackable backSpace;
//    List<VuforiaTrackable> allTrackables;
//    VuforiaTrackables targetsRoverRuckus;
    BlobDetector goldDetector = new BlobDetector(new Scalar(9, 100,50), new Scalar(38, 255,255), new Scalar(0,0,90), new Scalar(180, 20, 180), 5);
    BlobDetector silverDetector = new BlobDetector(new Scalar(0, 0,190), new Scalar(180, 40,255), new Scalar(0,0,50), new Scalar(180, 20, 160), 5);
    int NUM_FRAMES_CONSIDERED = 5;
    int NUM_TIME_RESAMPLED = 0;
    double wheelWidthBetweenWheels = 222.25;
    double wheelHeighBetweenWheels = 355;
    double distanceToTravel = 2*Math.PI*Math.sqrt(Math.pow(wheelHeighBetweenWheels/2,2)+Math.pow(wheelWidthBetweenWheels/2,2))*180/360;
    final double     COUNTS_PER_MOTOR_REV = 1440 ;    // eg: TETRIX Motor Encoder
    final double     DRIVE_GEAR_REDUCTION = 0.5 ;     // This is < 1.0 if geared UP
    final double     WHEEL_DIAMETER_MM = 100.0 ;     // For figuring circumference
    final double     COUNTS_PER_MM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_MM * 3.1415);

    @Override
    public void runOpMode() {
        initalizeRobot();
        waitForStart();
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        unlatch robot
//        lift.setTargetPosition((int)(COUNTS_PER_MOTOR_REV*20/60*5.7));
        lift.setPower(1);
        sleep(500);
        markerDropper.setPosition(0.75); // if 1 is up and 0 is down
        lift.setPower(0);
        sleep(4000);
        lift.setPower(0.5);
        sleep(200);
        lift.setPower(0);
        markerDropper.setPosition(1);
        DriveTrain.mecanum(baseMotorArray, 0.4,0,0,true);
        sleep(700);
        lift.setPower(-1);
        DriveTrain.mecanum(baseMotorArray, 0,0.4,0,true);
        sleep(300);
        lift.setPower(0);
        DriveTrain.mecanum(baseMotorArray,-0.4,0,0,true);
        sleep(700);
        DriveTrain.mecanum(baseMotorArray,0,-0.4,0,true);
        sleep(300);
        DriveTrain.mecanum(baseMotorArray,0,0,0,true);
        sleep(500);
        double[] goldAtPos1 = lookForMineral(MineralType.Gold);
        // if gold is straight ahead
        if (goldAtPos1[0] >= 0 && goldAtPos1[1] >= 0 && goldAtPos1[2] >= 0){
            // center on the gold
//            centerOnGold();
            moveForwardByDistance(170,1);
//            DriveTrain.mecanum(baseMotorArray, 0, 1, 0, true);
//            sleep(3000);
//            DriveTrain.mecanum(baseMotorArray,0,0,0,true);
        }else {
//            DriveTrain.mecanum(baseMotorArray, -1, 0, 0, true);
//            sleep(100);
//            DriveTrain.mecanum(baseMotorArray, 0, 0, 0, true);
//            telemetry.addLine("backToStart");
//            telemetry.update();
//            sleep(3000);
//            double[] silverPos = lookForMineral(MineralType.Silver);
//            if (silverPos[0] > 288/2){
//                DriveTrain.mecanum(baseMotorArray, -0.5, 0, 0, true);
//                sleep(100);
//                DriveTrain.mecanum(baseMotorArray, 0, 0, 0, true);
//            }else {
//                DriveTrain.mecanum(baseMotorArray, 0.5, 0, 0, true);
//                sleep(100);
//                DriveTrain.mecanum(baseMotorArray, 0, 0, 0, true);
//            }
            DriveTrain.turn(baseMotorArray, 40, wheelWidthBetweenWheels, wheelHeighBetweenWheels);
            sleep(500);
//            DriveTrain.mecanum(baseMotorArray, 0,0,1,true);
//            sleep(400);
//            DriveTrain.mecanum(baseMotorArray, 0,0,0,true);
            setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            double[] goldAtPos2 = lookForMineral(MineralType.Gold);
            // if gold is to the right
            if (goldAtPos2[0] >= 0 && goldAtPos2[1] >= 0 && goldAtPos2[2] >=0){
                moveForwardByDistance(156.5, 1);
                setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                centerOnGold();
//                DriveTrain.mecanum(baseMotorArray, 0, 1, 0, true);
//                sleep(2000);
                moveByEncoder(3840,-1,0.3);
                moveByEncoder(200, 1,0);
//                DriveTrain.mecanum(baseMotorArray, -1,0,0,true);
//                sleep(1200);
//                DriveTrain.mecanum(baseMotorArray,0,0,0,true);
                // drop marker


            }else {
//                DriveTrain.mecanum(baseMotorArray, 0,0,-1,true);
//                sleep(600);
//                DriveTrain.mecanum(baseMotorArray, 0,0,0,true);
                DriveTrain.turn(baseMotorArray, -100, wheelWidthBetweenWheels, wheelHeighBetweenWheels);
//                sleep(500);
//                centerOnGold();
                moveForwardByDistance(156.5, 1);
                setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
                moveByEncoder(5040,1,0);
//                DriveTrain.mecanum(baseMotorArray, 1,0, 0,true);
//                sleep(1500);
//                DriveTrain.mecanum(baseMotorArray,0,0,0,true);
//                markerDropper.setPosition(1);
//                double[] goldAtPos3 = lookForMineral(MineralType.Gold);
//                if (goldAtPos3[0] < 0 && goldAtPos3[1] < 0 && goldAtPos3[2] < 0 && NUM_TIME_RESAMPLED < 2) {
//                    NUM_TIME_RESAMPLED++;
//                    sample();
//                }else {

//                }
            }
        }
        markerDropper.setPosition(0);
        sleep(500);

////        sample();
//        // stuff to check gold detection with robot
//        setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        double[] goldAtPos1 = lookForMineral(MineralType.Gold);
//        DriveTrain.mecanum(baseMotorArray, 0,0,1,true);
////        DriveTrain.turn(baseMotorArray, 45, 5, 5);
//        sleep(400);
//        DriveTrain.mecanum(baseMotorArray, 0,0,0,true);
//        setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        double[] goldAtPos2 = lookForMineral(MineralType.Gold);
////        DriveTrain.turn(baseMotorArray, -90, 5, 5);
//        DriveTrain.mecanum(baseMotorArray, 0,0,-1,true);
//        sleep(700);
//        DriveTrain.mecanum(baseMotorArray, 0,0,0,true);
//        setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        double[] goldAtPos3 = lookForMineral(MineralType.Gold);
//        telemetry.addData("goldAtPos1 "+ goldAtPos1[0] + " " + goldAtPos1[1] + " ", goldAtPos1[2]);
//        telemetry.addData("goldAtPos2 "+ goldAtPos2[0] + " " + goldAtPos2[1] + " ", goldAtPos2[2]);
//        telemetry.addData("goldAtPos3 "+ goldAtPos3[0] + " " + goldAtPos3[1] + " ", goldAtPos3[2]);
//        telemetry.update();
//        sleep(5000);


////        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters.cameraDirection = CAMERA_CHOICE;
//        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
//        targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
//        blueRover = targetsRoverRuckus.get(0);
//        blueRover.setName("Blue-Rover");
//        redFootprint = targetsRoverRuckus.get(1);
//        redFootprint.setName("Red-Footprint");
//        frontCraters = targetsRoverRuckus.get(2);
//        frontCraters.setName("Front-Craters");
//        backSpace = targetsRoverRuckus.get(3);
//        backSpace.setName("Back-Space");
//
//        allTrackables = new ArrayList<>();
//        allTrackables.addAll(targetsRoverRuckus);
//        waitForStart();
//        getPos();

    }

    private void centerOnGold() {
        // 6.5in for 5.5cm diameter wheels
        //101 pixels per 6 in at 2ft 10 in away
        for (int i = 0; i < 5;i++) {
            DriveTrain.mecanum(baseMotorArray, 0, 0, 0, true);
//        setMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(300);
            double[] goldPos = lookForMineral(MineralType.Gold);

            double xPower;
            if (FtcRobotControllerActivity.frameSize != null) {
                xPower = (goldPos[0] - FtcRobotControllerActivity.frameSize.width / 2) / 50;
            }else{
                xPower = (goldPos[0] - 240 / 2) / 50;
            }
            DriveTrain.mecanum(baseMotorArray, -xPower, 0, 0, true);
            sleep((long) Math.abs(xPower * 60));
            DriveTrain.mecanum(baseMotorArray, 0, 0, 0, true);
        }
    }

    private void moveForwardByDistance(double distance, double power) {
//            distance is in cm and always positive
        final double     COUNTS_PER_MOTOR_REV = 1440 ;    // eg: TETRIX Motor Encoder
        final double     DRIVE_GEAR_REDUCTION = 0.5 ;     // This is < 1.0 if geared UP
        final double     WHEEL_DIAMETER_MM = 100.0 ;     // For figuring circumference
        final double     COUNTS_PER_MM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_MM * 3.1415);
        for (Object baseMotorObject: baseMotorArray){
            DcMotor baseMotor = (DcMotor) baseMotorObject;
            baseMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            baseMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            baseMotor.setPower(power);

        }
        while (COUNTS_PER_MM*10*distance > Math.abs(((DcMotor)baseMotorArray.get(0)).getCurrentPosition())){

        }
        DriveTrain.mecanum(baseMotorArray,0,0,0,true);
    }


    private void setMotorRunMode(DcMotor.RunMode runMode) {
        for (Object baseMotorObject: baseMotorArray){
            DcMotor baseMotor = (DcMotor) baseMotorObject;
            baseMotor.setMode(runMode);
        }
    }

    private double[] lookForMineral(MineralType mineralType){
        FtcRobotControllerActivity.shouldProcessImage = true;
        while (FtcRobotControllerActivity.shouldProcessImage) { // should process image is turned to false after the image is processed
            sleep(5);
        }
        ArrayList<double[]> frameOneCandidates;
        if (mineralType == MineralType.Gold) {
            frameOneCandidates = goldDetector.getCandidatesData();
        }else {
            frameOneCandidates = silverDetector.getCandidatesData();
        }

        DriveTrain.mecanum(baseMotorArray, 0.4, 0, 0, true);
        sleep(200);
        DriveTrain.mecanum(baseMotorArray, 0, 0, 0, true);
        sleep(300);
        FtcRobotControllerActivity.shouldProcessImage = true;
        while (FtcRobotControllerActivity.shouldProcessImage) { // should process image is turned to false after the image is processed
            sleep(5);
        }
        ArrayList<double[]> frameTwoCandidates;
        if (mineralType == MineralType.Gold) {
            frameTwoCandidates = goldDetector.getCandidatesData();
        }else {
            frameTwoCandidates = silverDetector.getCandidatesData();
        }
        ArrayList<Pair<double[], double[]>> pairs = findPairs(frameOneCandidates, frameTwoCandidates);
        ArrayList<Pair<double[], double[]>> acceptablePairs = new ArrayList<>();
        double MIN_HORIZONTAL_CHANGE = 0;
        for (Pair<double[], double[]> pair: pairs){
            if (pair.first[2] > 600 && pair.second[2] > 600 && pair.first[2] < 5000 && pair.second[2] < 5000 &&
                    isAboutEqual(pair.first[2],pair.second[2], 3000) &&
                    isAboutEqual(pair.first[1], pair.second[1], 10) && pair.first[0] - MIN_HORIZONTAL_CHANGE > pair.second[0]) {
                acceptablePairs.add(pair);
            }
        }
        telemetry.addData("length of acceptible size array", acceptablePairs.size());
        try {
            Pair<double[], double[]> farthestPairInXDirection = acceptablePairs.get(0);
            for (Pair<double[], double[]> pair: acceptablePairs){
                if (Math.abs(pair.first[0]-pair.second[0]) >
                        Math.abs(farthestPairInXDirection.first[0]-farthestPairInXDirection.second[0])){
                    farthestPairInXDirection = pair;
                }
            }
            telemetry.addData("gold At y1: " + farthestPairInXDirection.first[1] +" y2:" +farthestPairInXDirection.second[1] + " size1: ", farthestPairInXDirection.first[2]);
            telemetry.addLine("x1: " + farthestPairInXDirection.first[0] + " x2: " + farthestPairInXDirection.second[0]);
            telemetry.update();
            DriveTrain.mecanum(baseMotorArray, -0.4, 0, 0, true);
            sleep(200);
            DriveTrain.mecanum(baseMotorArray, 0, 0, 0, true);
            sleep(300);
            return farthestPairInXDirection.first;
        }catch (IndexOutOfBoundsException e){
            telemetry.addLine("no gold");
            telemetry.update();
            return new double[] {-1, -1,-1};
        }
    }

    
    
    private ArrayList<Pair<double[], double[]>> findPairs(ArrayList<double[]> frame1Data, ArrayList<double[]> frame2Data) {
        ArrayList<Pair<double[], double[]>> matches = new ArrayList<>();
        for (double[] oneCandidateData:frame1Data){
            double[] bestMatch = new double[]{-100,-100,-100};

            for (double[] aSecondCandidateData:frame2Data){
                // if ys and sizes are about equal
                if (Math.abs(aSecondCandidateData[1] - oneCandidateData[1]) < Math.abs(oneCandidateData[1] - bestMatch[1]) &&
                        Math.abs(aSecondCandidateData[2] - oneCandidateData[2]) < Math.abs(oneCandidateData[2] - bestMatch[2])){
                    bestMatch = aSecondCandidateData;
                }
//                if (isAboutEqual(oneCandidateData[1],aSecondCandidateData[1], 40) &&
//                        isAboutEqual(oneCandidateData[2],aSecondCandidateData[2], 500)){
//                    matches.add(new Pair<double[], double[]>(oneCandidateData, aSecondCandidateData));
//                }
            }
            matches.add(new Pair<double[], double[]>(oneCandidateData, bestMatch));
        }
        return matches;
    }

    private boolean isAboutEqual(double lhs, double rhs, double leniency){
        return lhs + leniency/2 > rhs && lhs - leniency/2 < rhs;
    }

    private void initalizeRobot(){
        baseMotorArray.add(hardwareMap.dcMotor.get("motorLF"));
        baseMotorArray.add(hardwareMap.dcMotor.get("motorRF"));
        baseMotorArray.add(hardwareMap.dcMotor.get("motorLB"));
        baseMotorArray.add(hardwareMap.dcMotor.get("motorRB"));
        ((DcMotor) baseMotorArray.get(1)).setDirection(DcMotor.Direction.REVERSE);
        ((DcMotor) baseMotorArray.get(3)).setDirection(DcMotor.Direction.REVERSE);
        lift = hardwareMap.dcMotor.get("lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        markerDropper = hardwareMap.servo.get("dropper");
//        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private enum MineralType {
        Gold, Silver, None
    }

    private void moveByEncoder(int encoderDistance, double x, double y){
        DriveTrain.mecanum(baseMotorArray,0,0,0,true);
        for (Object baseMotorObject: baseMotorArray){
            DcMotor baseMotor = (DcMotor) baseMotorObject;
            baseMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            baseMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        DriveTrain.mecanum(baseMotorArray,x,y,0,true);
        while (encoderDistance > Math.abs(((DcMotor)baseMotorArray.get(0)).getCurrentPosition())){

        }
        DriveTrain.mecanum(baseMotorArray,0,0,0,true);
    }
}
