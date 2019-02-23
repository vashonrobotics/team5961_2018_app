package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.opencv.core.Scalar;


import java.util.ArrayList;

import static com.qualcomm.robotcore.util.Range.clip;

@Autonomous(group = "5961", name = "Full Crater Autonomous")
public class CraterAutonomous extends LinearOpMode {
    private BNO055IMU imu;
    private ArrayList baseMotorArray = new ArrayList();
    private DcMotor lift;
    private Servo markerDropper;
    private Servo liftLock;
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
    private BlobDetector goldDetector = new BlobDetector(new Scalar(9, 100,50), new Scalar(38, 255,255));
    private BlobDetector silverDetector = new BlobDetector(new Scalar(0, 0,190), new Scalar(180, 40,255));
    private IMUWallImpactDetector imuWallImpactDetector = new IMUWallImpactDetector(telemetry,new JustLoggingAccelerationIntegrator());
    int NUM_FRAMES_CONSIDERED = 5;
    int NUM_TIME_RESAMPLED = 0;
    double wheelWidthBetweenWheels = 395.44;//215;
    double wheelHeighBetweenWheels = 0;//340;
    double distanceToTravel = 2*Math.PI*Math.sqrt(Math.pow(wheelHeighBetweenWheels/2,2)+Math.pow(wheelWidthBetweenWheels/2,2))*180/360;
    final double     COUNTS_PER_MOTOR_REV = 1440 ;    // eg: TETRIX Motor Encoder
    final double     DRIVE_GEAR_REDUCTION = 0.5 ;     // This is < 1.0 if geared UP
    final double     WHEEL_DIAMETER_MM = 100.0 ;     // For figuring circumference
    final double     COUNTS_PER_MM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_MM * 3.1415);
    
    @Override
    public void runOpMode(){
        try {
            telemetry.addLine("program started");
            telemetry.update();
            long start = System.nanoTime();
            initalizeRobot();
            telemetry.addData("time to initialize", System.nanoTime()-start);
            telemetry.update();
            waitForStart();
//            moveByEncoder(2000,1,1,false);
//            safeSleep(5000000);
            liftLock.setPosition(1);
            sleep(400);
            markerDropper.setPosition(0.9);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        unlatch robot
//        lift.setTargetPosition((int)(COUNTS_PER_MOTOR_REV*20/60*5.7));
            lift.setPower(-1);
            safeSleep(500);
            lift.setPower(0);
            safeSleep(1000);
            telemetry.addLine("slept 1000");
            telemetry.update();
            safeSleep(1000);
            telemetry.addLine("slept 2000");
            telemetry.update();
            safeSleep(500);
            telemetry.addLine("slept 2500");
            telemetry.update();
            long t1 = System.currentTimeMillis();
            telemetry.addLine("finished sleep for lower");
            telemetry.update();
            int craterHeight = (int) getCraterHeight();
            telemetry.addData("done getting craterHeight took ", System.currentTimeMillis()-t1);
            telemetry.update();
            lift.setPower(-1);
            safeSleep(200);
            lift.setPower(0);
            markerDropper.setPosition(1);
            DriveTrain.turn(baseMotorArray,-30,wheelWidthBetweenWheels,wheelHeighBetweenWheels);
            lift.setPower(1);
            safeSleep(1000);
            lift.setPower(0);
            DriveTrain.turn(baseMotorArray,30,wheelWidthBetweenWheels,wheelHeighBetweenWheels);
//            moveByEncoder(400,0.6,0,false);
//            DriveTrain.mecanum(baseMotorArray, 0.4, 0, 0, true);
//            safeSleep(600);
//            lift.setPower(1);
//            moveByEncoder(400,0,0.6,false);
//            DriveTrain.mecanum(baseMotorArray, 0, 0.4, 0, true);
//            safeSleep(400);
//            DriveTrain.mecanum(baseMotorArray, -0.4, 0, 0, true);
//            safeSleep(600);
//            lift.setPower(0);
//            moveByEncoder(400,-.6,0,false);
//
//            moveByEncoder(400,0,-0.6,false);
//            DriveTrain.mecanum(baseMotorArray, 0, -0.4, 0, true);
//            safeSleep(400);
//            DriveTrain.mecanum(baseMotorArray, 0, 0, 0, true);
//            DriveTrain.turn(baseMotorArray,30,wheelWidthBetweenWheels,wheelHeighBetweenWheels);
//            safeSleep(100);
            DriveTrain.turn(baseMotorArray,5,wheelWidthBetweenWheels,wheelHeighBetweenWheels);
//        safeSleep(500);
//        int craterHeight = (int)getCraterHeight();
//        safeSleep(1000);
            BlobDetectorCandidate goldAtPos1 = lookForMineral(MineralType.Gold, craterHeight, FtcRobotControllerActivity.frameSize.width / 5, FtcRobotControllerActivity.frameSize.width / 5);
            // middle
            // x1 122 x2 92
            //x1 112 x2 77
            //x1 104 x2 86
            // if gold is straight ahead
            if (goldAtPos1.getX() >= 0 && goldAtPos1.getY() >= 0) {
                // center on the gold
                centerOnGold(FtcRobotControllerActivity.frameSize.width/2);
                moveForwardByDistance(100, 1);
//            safeSleep(100);
                moveForwardByDistance(100, -1);
//                DriveTrain.turn(baseMotorArray, -63, wheelWidthBetweenWheels, wheelWidthBetweenWheels);
                DriveTrain.turn(baseMotorArray, -45, wheelWidthBetweenWheels, wheelWidthBetweenWheels);

            } else {
                DriveTrain.turn(baseMotorArray, 30, wheelWidthBetweenWheels, wheelHeighBetweenWheels);
//            safeSleep(1000);
                BlobDetectorCandidate goldAtPos2 = lookForMineral(MineralType.Gold, craterHeight, 0, 0);
                telemetry.addData("gold pos 2", goldAtPos2.getY());
                // if gold is to the right
                if (goldAtPos2.getX() >= 0 && goldAtPos2.getY() >= 0) {
                    centerOnGold(FtcRobotControllerActivity.frameSize.width/3);
                    moveForwardByDistance(110, 1);
                    moveForwardByDistance(110, -1);
//                moveForwardByDistance(5,-0.5);
                    DriveTrain.turn(baseMotorArray, -75, wheelWidthBetweenWheels, wheelHeighBetweenWheels);

                } else {
//                DriveTrain.mecanum(baseMotorArray, 0,0,-1,true);
//                safeSleep(600);
//                DriveTrain.mecanum(baseMotorArray, 0,0,0,true);
                    DriveTrain.turn(baseMotorArray, -60, wheelWidthBetweenWheels, wheelHeighBetweenWheels);
//                safeSleep(500);
                    centerOnGold(FtcRobotControllerActivity.frameSize.width*2/3);
                    moveForwardByDistance(110, 1);
//                moveForwardByDistance(30, 0.5);
                    moveForwardByDistance(110, -1);
//                moveForwardByDistance(5,-0.5);
//                setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    DriveTrain.turn(baseMotorArray, -15, wheelWidthBetweenWheels, wheelHeighBetweenWheels);

//                moveByEncoder();
                }
            }
            moveForwardByDistance(160,1);
            moveByEncoder(800,-1,0,false);
//            turnToAngle(0, 10);
            moveForwardByDistance(180, 1);
//            turnToAngle(0, 10);
            moveByEncoder(4530, -1, 0,false);
            moveByEncoder(1030, -0.5, 0.5,false);//possibly unnecessary
            moveByEncoder(500, 1, 0, false);
            markerDropper.setPosition(0);
            safeSleep(500);
            moveByEncoder(1000, 1, -.1,false);
            DriveTrain.turn(baseMotorArray, -100, wheelWidthBetweenWheels, wheelHeighBetweenWheels);
//        /moveForwardByDistance(20,-1);
            moveByEncoder(500, 1, 0,false);
            moveForwardByDistanceWithoutRunToPosition(180, -1);
//            DriveTrain.turn(baseMotorArray, -10, wheelWidthBetweenWheels, wheelHeighBetweenWheels);
            moveByEncoder(4000, 0.8, -1,false);
//        moveForwardByDistanceWithoutRunToPosition(180,-0.5);
//        safeSleep(50000);
        }catch (Throwable e){
            System.out.println(e);

        }

    }

    private double getCraterHeight(){
        BlobDetector craterDetector = new BlobDetector(new Scalar(0, 0, 0), new Scalar (180, 50, 74));
        FtcRobotControllerActivity.shouldProcessImage = true;
        while (FtcRobotControllerActivity.shouldProcessImage) { // should process image is turned to false after the image is processed
            safeSleep(5);
        }
        ArrayList<BlobDetectorCandidate> craterCandidates = craterDetector.getCandidatesData(true);

        try {
            BlobDetectorCandidate crater = craterCandidates.get(0);
            for (BlobDetectorCandidate craterCandidate : craterCandidates) {
                if (craterCandidate.getWidth() > crater.getWidth()){
                    crater = craterCandidate;
                }
            }
            telemetry.addData("crater x: " + crater.getX() + " y: " +crater.getY()+ "width: " + crater.getWidth()+ "height:",crater.getHeight());

            if (crater.getY()+20 < FtcRobotControllerActivity.frameSize.height/2.8) {
                return crater.getY()+20;
            }else{
                return FtcRobotControllerActivity.frameSize.height/2.8;
            }
        } catch (IndexOutOfBoundsException e){
            return FtcRobotControllerActivity.frameSize.height/2.8;
        }
    }

    private void centerOnGold(double targetXValue) {
        // 6.5in for 5.5cm diameter wheels
        //101 pixels per 6 in at 2ft 10 in away
        double previousXOffset = 1000;
        double sleepReduceFactor = 1;
        for (int i = 0; i < 3;i++) {
            DriveTrain.mecanum(baseMotorArray, 0, 0, 0, true);
//        setMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            safeSleep(100);
            BlobDetectorCandidate goldPos = lookForMineral(MineralType.Gold,
                    FtcRobotControllerActivity.frameSize.height/2.8,0,0);
            telemetry.update();
            if (goldPos.getX() < 0 && goldPos.getY() < 0){
                telemetry.addLine("can't find gold");
                telemetry.update();
                break;
            }
            double xPower;
//            if (FtcRobotControllerActivity.frameSize != null) {
            xPower = (goldPos.getX() - targetXValue) / 40;

//            }else{
//                xPower = (goldPos.getX() - 240 / 2) / 50;
//            }
            DriveTrain.mecanum(baseMotorArray, xPower, 0, 0, true);
            if (Math.abs(goldPos.getX() - targetXValue) > Math.abs(previousXOffset)){
                sleepReduceFactor = sleepReduceFactor * 2 / 3;
            }
            previousXOffset = goldPos.getX() - targetXValue;
            safeSleep((int)(Math.abs(xPower * 60)*sleepReduceFactor));

            DriveTrain.mecanum(baseMotorArray, 0, 0, 0, true);
        }
    }

    private void moveForwardByDistance(double distance, double power) {
//            distance is in cm and always positive
        // for some reason dist*2/3 = actual dist
        final double     COUNTS_PER_MOTOR_REV = 1440 ;    // eg: TETRIX Motor Encoder
        final double     DRIVE_GEAR_REDUCTION = 0.5 ;     // This is < 1.0 if geared UP
        final double     WHEEL_DIAMETER_MM = 100.0 ;     // For figuring circumference
        final double     COUNTS_PER_MM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_MM * 3.1415);
        for (int i = 0; i < baseMotorArray.size(); i++){
            DcMotor baseMotor = (DcMotor) baseMotorArray.get(i);
            baseMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            baseMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            baseMotor.setPower(power/2);
            int directionMultiplier = (int) Math.signum(power);
//            if (i % 2 == 0){
//                directionMultiplier *= -1;
//            }
            baseMotor.setTargetPosition((int) (COUNTS_PER_MM*10*distance)*directionMultiplier);
        }

        for(int t = 5; t < 10; t++){
            DriveTrain.mecanum(baseMotorArray, 0, power*t/10, 0, true);
            sleep(50);
        }
        DriveTrain.mecanum(baseMotorArray,0,power,0,true);
        imuWallImpactDetector.setImpact(false);
        int encoderChange = 1000;
        int previousEncoderPosition = ((DcMotor)baseMotorArray.get(0)).getCurrentPosition();
        int numLooped = 0;
        while ((Math.abs(encoderChange) > 5 || numLooped < 3) && opModeIsActive()){
            checkForOpModeActivity();
            numLooped++;
            safeSleep(30);
            encoderChange = ((DcMotor)baseMotorArray.get(0)).getCurrentPosition() - previousEncoderPosition;
            previousEncoderPosition = ((DcMotor)baseMotorArray.get(0)).getCurrentPosition();
        }
        safeSleep(100);
//        while (COUNTS_PER_MM*10*distance > Math.abs(((DcMotor)baseMotorArray.get(0)).getCurrentPosition())){
//
//        }
        DriveTrain.mecanum(baseMotorArray,0,0,0,true);
        setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveTrain.mecanum(baseMotorArray,0,0,0,true);
    }


    private void setMotorRunMode(DcMotor.RunMode runMode) {
        for (Object baseMotorObject: baseMotorArray){
            DcMotor baseMotor = (DcMotor) baseMotorObject;
            baseMotor.setMode(runMode);
        }
    }

    private BlobDetectorCandidate lookForMineral(MineralType mineralType, double maxY, double XBorderWidthLeft, double XBorderWidthRight){
//        safeSleep(500);
        FtcRobotControllerActivity.shouldProcessImage = true;
        while (FtcRobotControllerActivity.shouldProcessImage&& opModeIsActive()) { // should process image is turned to false after the image is processed
            checkForOpModeActivity();
            safeSleep(5);
        }
        ArrayList<BlobDetectorCandidate> frameOneCandidates;
        if (mineralType == MineralType.Gold) {
            frameOneCandidates = goldDetector.getCandidatesData(false);
        }else {
            frameOneCandidates = silverDetector.getCandidatesData(false);
        }



        ArrayList<BlobDetectorCandidate> blobs = frameOneCandidates;
        ArrayList<BlobDetectorCandidate> acceptableBlobs = new ArrayList<>();
        double MIN_HORIZONTAL_CHANGE = 0;
        for (BlobDetectorCandidate blob: blobs){
            double sizeOfFirst = blob.getWidth()*blob.getHeight();// size of bounding rect
            double sizeOfSecond = blob.getWidth()*blob.getHeight();
            if (sizeOfFirst > 500 && sizeOfSecond > 500 && sizeOfFirst < 5000 && sizeOfSecond < 5000 &&
                    isAboutEqual(sizeOfFirst,sizeOfSecond, 1000) &&
                    isAboutEqual(blob.getY(), blob.getY(), 10) &&
                    blob.getY() <= maxY && blob.getX() > XBorderWidthLeft && blob.getX() < FtcRobotControllerActivity.frameSize.width - XBorderWidthRight) {
                acceptableBlobs.add(blob);
            }
        }
        telemetry.addData("length of acceptible size array", acceptableBlobs.size());
//        moveByEncoder(200, -0.4,0);
        try {
            BlobDetectorCandidate farthestBlobInXDirection = acceptableBlobs.get(0);
            for (BlobDetectorCandidate blob: acceptableBlobs){
                if (Math.abs(blob.getX()-blob.getX()) >
                        Math.abs(farthestBlobInXDirection.getX()-farthestBlobInXDirection.getX())){
                    farthestBlobInXDirection = blob;
                }
            }
            telemetry.addData("gold At y1: " + farthestBlobInXDirection.getY() +" y2:" +farthestBlobInXDirection.getY() + " size1: ", farthestBlobInXDirection.getWidth()*farthestBlobInXDirection.getHeight());
            telemetry.addLine("x1: " + farthestBlobInXDirection.getX() + " x2: " + farthestBlobInXDirection.getX());
            telemetry.update();
            return farthestBlobInXDirection;
        }catch (IndexOutOfBoundsException e){
            telemetry.addLine("no gold");
            telemetry.update();
            return new BlobDetectorCandidate(-1, -1,-1,-1);
        }
    }



    private ArrayList<Pair<BlobDetectorCandidate, BlobDetectorCandidate>> findPairs(ArrayList<BlobDetectorCandidate> frame1Data, ArrayList<BlobDetectorCandidate> frame2Data) {
        ArrayList<Pair<BlobDetectorCandidate, BlobDetectorCandidate>> matches = new ArrayList<>();
        for (BlobDetectorCandidate oneCandidateData:frame1Data){
            BlobDetectorCandidate bestMatch = new BlobDetectorCandidate(-100,-100,-100,-100);

            for (BlobDetectorCandidate aSecondCandidateData:frame2Data){
                // if ys and sizes are about equal
                double sizeOfFirstCandidate = oneCandidateData.getWidth()*oneCandidateData.getHeight();// size of bounding rect
                double sizeOfSecondCandidate = aSecondCandidateData.getWidth()*aSecondCandidateData.getHeight();
                if (Math.abs(aSecondCandidateData.getY() - oneCandidateData.getY()) < Math.abs(oneCandidateData.getY() - bestMatch.getY()) &&
                        Math.abs(sizeOfSecondCandidate - sizeOfFirstCandidate) < Math.abs(sizeOfFirstCandidate - (bestMatch.getWidth()*bestMatch.getHeight()))){
                    bestMatch = aSecondCandidateData;
                }
//                if (isAboutEqual(oneCandidateData[1],aSecondCandidateData[1], 40) &&
//                        isAboutEqual(oneCandidateData[2],aSecondCandidateData[2], 500)){
//                    matches.add(new Pair<BlobDetectorCandidate, BlobDetectorCandidate>(oneCandidateData, aSecondCandidateData));
//                }
            }
            matches.add(new Pair<BlobDetectorCandidate,BlobDetectorCandidate>(oneCandidateData, bestMatch));
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
        liftLock = hardwareMap.servo.get("liftLock");
        imu = hardwareMap.get(BNO055IMU.class,"imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = imuWallImpactDetector;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(),100);
        for(int i = 0; i < 4; i++){
            ((DcMotor)baseMotorArray.get(i)).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }


//        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private enum MineralType {
        Gold, Silver, None
    }

    private void moveByEncoder(int encoderDistance, double x, double y, boolean stopWithBumpDetection){
        DriveTrain.mecanum(baseMotorArray,0,0,0,true);
        double power = clip(Math.sqrt((x * x) + (y * y)),-1.0,1.0);
        double beginningAngle = imu.getAngularOrientation().firstAngle;

        double radianAngle = Math.atan2(y, x) - Math.PI * 1/4;
        double[] motorPowers = {(Math.cos(radianAngle) * power), // frontLeft
                (Math.sin(radianAngle) * power), // frontRight
                (Math.sin(radianAngle) * power),// backLeft
                (Math.cos(radianAngle) * power)};
        double totalPower = 0;
        for(double motorPower:motorPowers){
            totalPower += motorPower;
        }
        for(int i = 0; i < baseMotorArray.size(); i++) {
            DcMotor motor = ((DcMotor) baseMotorArray.get(i));
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            int sideMultiplier = i % 2 == 0 ? 1 : -1;

            motor.setTargetPosition((int) (encoderDistance*motorPowers[i]*2));
//            motor.setPower(0.4*sideMultiplier*Math.signum(angle));
        }
        for(int t = 5; t < 10; t++){
            DriveTrain.mecanum(baseMotorArray, 0, Math.abs(power)*t/10, 0, true);
            sleep(50);
        }
        DriveTrain.mecanum(baseMotorArray,0,Math.abs(power),0,true);
        imuWallImpactDetector.setImpact(false);
        int encoderChange = 1000;
        int previousEncoderPosition = ((DcMotor)baseMotorArray.get(0)).getCurrentPosition();
        int numTimes_looped = 0;
        while ((Math.abs(encoderChange) > 5 || numTimes_looped < 3)&&opModeIsActive() && !(imuWallImpactDetector.isImpact()&&stopWithBumpDetection)) {
            checkForOpModeActivity();
            numTimes_looped++;
            sleep(30);
            encoderChange = ((DcMotor) baseMotorArray.get(0)).getCurrentPosition() - previousEncoderPosition;
            previousEncoderPosition = ((DcMotor) baseMotorArray.get(0)).getCurrentPosition();
        }
        sleep(100);
        DriveTrain.mecanum(baseMotorArray,0,0,0,true);
        if (y < 0.1 && y > -0.1) {
//            double angleToTurn = -(imu.getAngularOrientation().firstAngle * Math.signum(beginningAngle) + beginningAngle);
            double angleToTurn = beginningAngle - imu.getAngularOrientation().firstAngle;
//            DriveTrain.turn(baseMotorArray, angleToTurn, wheelWidthBetweenWheels, wheelHeighBetweenWheels);
//            angleToTurn = -(imu.getAngularOrientation().firstAngle * Math.signum(beginningAngle) + beginningAngle);
//            DriveTrain.turn(baseMotorArray, angleToTurn, wheelWidthBetweenWheels, wheelHeighBetweenWheels);
        }
    }
    private void safeSleep(int sleepTime) {
//        sleep(sleepTime);
        try {
            Thread.sleep(sleepTime);
        }catch (InterruptedException e){
            throw new RuntimeException(e);

//            System.out.println(e);
//            DriveTrain.mecanum(baseMotorArray,0,0,0,true);
        }
    }
    private void moveForwardByDistanceWithoutRunToPosition(double distance, double power) {
//            distance is in cm and always positive
        // goes to the right slightly
        final double     COUNTS_PER_MOTOR_REV = 1440 ;    // eg: TETRIX Motor Encoder
        final double     DRIVE_GEAR_REDUCTION = 0.5 ;     // This is < 1.0 if geared UP
        final double     WHEEL_DIAMETER_MM = 100.0 ;     // For figuring circumference
        final double     COUNTS_PER_MM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_MM * 3.1415);
        for (Object baseMotorObject: baseMotorArray){
            DcMotor baseMotor = (DcMotor) baseMotorObject;
            baseMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            baseMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
        DriveTrain.mecanum(baseMotorArray,.2,power,0,true);
        while (COUNTS_PER_MM*10*distance > Math.abs(((DcMotor)baseMotorArray.get(0)).getCurrentPosition())&&opModeIsActive()){
            checkForOpModeActivity();
        }
        DriveTrain.mecanum(baseMotorArray,0,0,0,true);
    }

    private void checkForOpModeActivity() {
        if(!opModeIsActive()) {
            throw new RuntimeException("opmode no longer active");
        }
    }

    void turnToAngle(double desiredAngle, double turnLimit){
//        telemetry.addData("angle before",imu.getAngularOrientation().firstAngle);
//        telemetry.update();
//        safeSleep(1000);
//        telemetry.addData("angle after",imu.getAngularOrientation().firstAngle);
//        telemetry.update();
        // angle in degrees. clock-wise is positive
        // angle where the lander position is 45 degrees clockwise
        double angleToTurn = imu.getAngularOrientation().firstAngle+desiredAngle-45;
        if (angleToTurn > turnLimit){
            angleToTurn = turnLimit;
        }else if (angleToTurn < -turnLimit){
            angleToTurn = -turnLimit;
        }
        DriveTrain.turn(baseMotorArray,angleToTurn,wheelWidthBetweenWheels, wheelHeighBetweenWheels);
        angleToTurn = imu.getAngularOrientation().firstAngle+desiredAngle-45;
        if (angleToTurn > turnLimit){
            angleToTurn = turnLimit;
        }else if (angleToTurn < -turnLimit){
            angleToTurn = -turnLimit;
        }
        DriveTrain.turn(baseMotorArray,angleToTurn,wheelWidthBetweenWheels, wheelHeighBetweenWheels);

    }
}
