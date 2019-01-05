package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

import java.util.ArrayList;

import static com.qualcomm.robotcore.util.Range.clip;

@Autonomous(group = "5961", name = "Full Depot Autonomous")
public class DepotAutonomous extends LinearOpMode {
    private ArrayList baseMotorArray = new ArrayList();
    private DcMotor lift;
    private Servo markerDropper;
    private BNO055IMU imu;
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
    BlobDetector goldDetector = new BlobDetector(new Scalar(9, 100,50), new Scalar(38, 255,255));
    BlobDetector silverDetector = new BlobDetector(new Scalar(0, 0,190), new Scalar(180, 40,255));
    IMUWallImpactDetector imuWallImpactDetector = new IMUWallImpactDetector(telemetry, new JustLoggingAccelerationIntegrator());
    int NUM_FRAMES_CONSIDERED = 5;
    int NUM_TIME_RESAMPLED = 0;
    double wheelWidthBetweenWheels = 215;
    double wheelHeighBetweenWheels = 340;
    double distanceToTravel = 2*Math.PI*Math.sqrt(Math.pow(wheelHeighBetweenWheels/2,2)+Math.pow(wheelWidthBetweenWheels/2,2))*180/360;
    final double     COUNTS_PER_MOTOR_REV = 1440 ;    // eg: TETRIX Motor Encoder
    final double     DRIVE_GEAR_REDUCTION = 0.5 ;     // This is < 1.0 if geared UP
    final double     WHEEL_DIAMETER_MM = 100.0 ;     // For figuring circumference
    final double     COUNTS_PER_MM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_MM * 3.1415);

    @Override
    public void runOpMode() {
        initalizeRobot();
//        DriveTrain.turn(baseMotorArray, -90, wheelWidthBetweenWheels, wheelHeighBetweenWheels);

        waitForStart();
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        unlatch robot
//        lift.setTargetPosition((int)(COUNTS_PER_MOTOR_REV*20/60*5.7));
        lift.setPower(1);
        safeSleep(500);
        markerDropper.setPosition(0.75); // if 1 is up and 0 is down
        lift.setPower(0);
        safeSleep(4000);
        lift.setPower(1);
        safeSleep(200);
        lift.setPower(0);
        markerDropper.setPosition(1);
        DriveTrain.mecanum(baseMotorArray, 0.4,0,0,true);
        safeSleep(700);
        lift.setPower(-1);
        DriveTrain.mecanum(baseMotorArray, 0,0.4,0,true);
        safeSleep(300);
        DriveTrain.mecanum(baseMotorArray,-0.4,0,0,true);
        safeSleep(700);
        lift.setPower(0);
        DriveTrain.mecanum(baseMotorArray,0,-0.4,0,true);
        safeSleep(300);
        DriveTrain.mecanum(baseMotorArray,0,0,0,true);
//        DriveTrain.turn(baseMotorArray,8,wheelWidthBetweenWheels,wheelHeighBetweenWheels);
        BlobDetectorCandidate goldAtPos1 = lookForMineral(MineralType.Gold,FtcRobotControllerActivity.frameSize.height/3,FtcRobotControllerActivity.frameSize.width/5,FtcRobotControllerActivity.frameSize.width/5);
        // if gold is straight ahead
        if (goldAtPos1.getX() >= 0 && goldAtPos1.getY() >= 0){
            // center on the gold
//            centerOnGold();
            moveForwardByDistance(195,1);
            moveByEncoder(1000,1,0);
            DriveTrain.turn(baseMotorArray,-45,wheelWidthBetweenWheels,wheelHeighBetweenWheels);
            moveByEncoder(1000,1,0);


        }else {
            DriveTrain.turn(baseMotorArray, 30, wheelWidthBetweenWheels, wheelHeighBetweenWheels);
            setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BlobDetectorCandidate goldAtPos2 = lookForMineral(MineralType.Gold,FtcRobotControllerActivity.frameSize.height/2.5,0, 0);
            // if gold is to the right
            if (goldAtPos2.getX() >= 0 && goldAtPos2.getY() >= 0){
                moveForwardByDistance(156.5, 1);
                setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
                DriveTrain.turn(baseMotorArray,-70,wheelWidthBetweenWheels,wheelHeighBetweenWheels);
                moveByEncoder(500,1,0);
//                centerOnGold();
//                DriveTrain.mecanum(baseMotorArray, 0, 1, 0, true);
//                safeSleep(2000);
                moveByEncoder(2040,0,1);
//                DriveTrain.mecanum(baseMotorArray, -1,0,0,true);
//                safeSleep(1200);
//                DriveTrain.mecanum(baseMotorArray,0,0,0,true);
                // drop marker


            }else {
//                DriveTrain.mecanum(baseMotorArray, 0,0,-1,true);
//                safeSleep(600);
//                DriveTrain.mecanum(baseMotorArray, 0,0,0,true);
                DriveTrain.turn(baseMotorArray, -60, wheelWidthBetweenWheels, wheelHeighBetweenWheels);
//                safeSleep(500);
//                centerOnGold();
                moveForwardByDistance(156.5, 1);
                DriveTrain.turn(baseMotorArray,-20, wheelWidthBetweenWheels,wheelHeighBetweenWheels);
//                moveForwardByDistance(5,1);
//                DriveTrain.mecanum(baseMotorArray,-0.5,0,0,true);
//                safeSleep(1000);
                setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
                moveByEncoder(3540,1,0.0);
//                moveByEncoder(500,0.7,0);
//                DriveTrain.mecanum(baseMotorArray, 1,0, 0,true);
//                safeSleep(1500);
//                DriveTrain.mecanum(baseMotorArray,0,0,0,true);
//                markerDropper.setPosition(1);
//                BlobDetectorCandidate goldAtPos3 = lookForMineral(MineralType.Gold);
//                if (goldAtPos3[0] < 0 && goldAtPos3[1] < 0 && goldAtPos3[2] < 0 && NUM_TIME_RESAMPLED < 2) {
//                    NUM_TIME_RESAMPLED++;
//                    sample();
//                }else {

//                }
            }
        }
        moveByEncoder(2000,0,1);
        moveByEncoder(700,0.5,0);
        moveByEncoder(500,-1,-1);
        DriveTrain.turn(baseMotorArray,90,wheelWidthBetweenWheels,wheelHeighBetweenWheels);
        markerDropper.setPosition(0);
        safeSleep(700);
        moveByEncoder(1000,0,-1);
        DriveTrain.turn(baseMotorArray,-181,wheelWidthBetweenWheels,wheelHeighBetweenWheels);

        moveByEncoder(500,0.9,0);
        turnToAngle(180);
        moveForwardByDistanceWithoutRunToPosition(190,1);
        moveForwardByDistanceWithoutRunToPosition(180,0.5);

////        sample();
//        // stuff to check gold detection with robot
//        setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        BlobDetectorCandidate goldAtPos1 = lookForMineral(MineralType.Gold);
//        DriveTrain.mecanum(baseMotorArray, 0,0,1,true);
////        DriveTrain.turn(baseMotorArray, 45, 5, 5);
//        safeSleep(400);
//        DriveTrain.mecanum(baseMotorArray, 0,0,0,true);
//        setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        BlobDetectorCandidate goldAtPos2 = lookForMineral(MineralType.Gold);
////        DriveTrain.turn(baseMotorArray, -90, 5, 5);
//        DriveTrain.mecanum(baseMotorArray, 0,0,-1,true);
//        safeSleep(700);
//        DriveTrain.mecanum(baseMotorArray, 0,0,0,true);
//        setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        BlobDetectorCandidate goldAtPos3 = lookForMineral(MineralType.Gold);
//        telemetry.addData("goldAtPos1 "+ goldAtPos1[0] + " " + goldAtPos1[1] + " ", goldAtPos1[2]);
//        telemetry.addData("goldAtPos2 "+ goldAtPos2[0] + " " + goldAtPos2[1] + " ", goldAtPos2[2]);
//        telemetry.addData("goldAtPos3 "+ goldAtPos3[0] + " " + goldAtPos3[1] + " ", goldAtPos3[2]);
//        telemetry.update();
//        safeSleep(5000);


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
            safeSleep(300);
            BlobDetectorCandidate goldPos = lookForMineral(MineralType.Gold,1000,0,1);

            double xPower;
            if (FtcRobotControllerActivity.frameSize != null) {
                xPower = (goldPos.getX() - FtcRobotControllerActivity.frameSize.width / 2) / 50;
            }else{
                xPower = (goldPos.getX() - 240 / 2) / 50;
            }
            DriveTrain.mecanum(baseMotorArray, -xPower, 0, 0, true);
            safeSleep((int) Math.abs(xPower * 60));
            DriveTrain.mecanum(baseMotorArray, 0, 0, 0, true);
        }
    }
    private void moveForwardByDistance(double distance, double power) {
//            distance is in cm and always positive
        // for some reason dist*2/3 = actual dist
        final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
        final double DRIVE_GEAR_REDUCTION = 0.5;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_MM = 100.0;     // For figuring circumference
        final double COUNTS_PER_MM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_MM * 3.1415);
        for (int i = 0; i < baseMotorArray.size(); i++) {
            DcMotor baseMotor = (DcMotor) baseMotorArray.get(i);
            baseMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            baseMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            baseMotor.setPower(power);
            int directionMultiplier = (int) Math.signum(power);
//            if (i % 2 == 0){
//                directionMultiplier *= -1;
//            }
            baseMotor.setTargetPosition((int) (COUNTS_PER_MM * 10 * distance) * directionMultiplier);
        }
        safeSleep(300);
        int encoderChange = 1000;
        int previousEncoderPosition = ((DcMotor) baseMotorArray.get(0)).getCurrentPosition();
        int numLooped = 0;
        while ((Math.abs(encoderChange) > 5 || numLooped < 3) && opModeIsActive()) {
            numLooped++;
            safeSleep(30);
            encoderChange = ((DcMotor) baseMotorArray.get(0)).getCurrentPosition() - previousEncoderPosition;
            previousEncoderPosition = ((DcMotor) baseMotorArray.get(0)).getCurrentPosition();
        }
        safeSleep(100);
//        while (COUNTS_PER_MM*10*distance > Math.abs(((DcMotor)baseMotorArray.get(0)).getCurrentPosition())){
//
//        }
        DriveTrain.mecanum(baseMotorArray, 0, 0, 0, true);
        setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveTrain.mecanum(baseMotorArray, 0, 0, 0, true);
    }
    private void moveForwardByDistanceWithoutRunToPosition(double distance, double power) {
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
        while (COUNTS_PER_MM*10*distance > Math.abs(((DcMotor)baseMotorArray.get(0)).getCurrentPosition())&&opModeIsActive()){

        }
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
        while (FtcRobotControllerActivity.shouldProcessImage && opModeIsActive()) { // should process image is turned to false after the image is processed
            safeSleep(5);
        }
        ArrayList<BlobDetectorCandidate> frameOneCandidates;
        if (mineralType == MineralType.Gold) {
            frameOneCandidates = goldDetector.getCandidatesData(false);
        }else {
            frameOneCandidates = silverDetector.getCandidatesData(false);
        }


//        moveByEncoder(200, 0.4,0);
//        telemetry.addData("Encoder Distance: ",((DcMotor) baseMotorArray.get(0)).getCurrentPosition());
        FtcRobotControllerActivity.shouldProcessImage = true;
        while (FtcRobotControllerActivity.shouldProcessImage && opModeIsActive()) { // should process image is turned to false after the image is processed
            safeSleep(5);
        }
        ArrayList<BlobDetectorCandidate> frameTwoCandidates;
        if (mineralType == MineralType.Gold) {
            frameTwoCandidates = goldDetector.getCandidatesData(false);
        }else {
            frameTwoCandidates = silverDetector.getCandidatesData(false);
        }
        ArrayList<Pair<BlobDetectorCandidate,BlobDetectorCandidate>> pairs = findPairs(frameOneCandidates, frameTwoCandidates);
        ArrayList<Pair<BlobDetectorCandidate, BlobDetectorCandidate>> acceptablePairs = new ArrayList<>();
        double MIN_HORIZONTAL_CHANGE = 0;
        for (Pair<BlobDetectorCandidate, BlobDetectorCandidate> pair: pairs){
            double sizeOfFirst = pair.first.getWidth()*pair.first.getHeight();// size of bounding rect
            double sizeOfSecond = pair.second.getWidth()*pair.second.getHeight();
            if (sizeOfFirst > 300 && sizeOfSecond > 300 && sizeOfFirst < 5000 && sizeOfSecond < 5000 &&
                    isAboutEqual(sizeOfFirst,sizeOfSecond, 1000) &&
                    isAboutEqual(pair.first.getY(), pair.second.getY(), 10) &&
                    pair.first.getY() <= maxY && pair.second.getX() > XBorderWidthLeft && pair.second.getX() < FtcRobotControllerActivity.frameSize.width - XBorderWidthRight) {
                acceptablePairs.add(pair);
            }
        }
        telemetry.addData("length of acceptible size array", acceptablePairs.size());
//        moveByEncoder(200, -0.4,0);
        try {
            Pair<BlobDetectorCandidate, BlobDetectorCandidate> farthestPairInXDirection = acceptablePairs.get(0);
            for (Pair<BlobDetectorCandidate, BlobDetectorCandidate> pair: acceptablePairs){
                if (Math.abs(pair.first.getX()-pair.second.getX()) >
                        Math.abs(farthestPairInXDirection.first.getX()-farthestPairInXDirection.second.getX())){
                    farthestPairInXDirection = pair;
                }
            }
            telemetry.addData("gold At y1: " + farthestPairInXDirection.first.getY() +" y2:" +farthestPairInXDirection.second.getY() + " size1: ", farthestPairInXDirection.first.getWidth()*farthestPairInXDirection.first.getHeight());
            telemetry.addLine("x1: " + farthestPairInXDirection.first.getX() + " x2: " + farthestPairInXDirection.second.getX());
            telemetry.update();
            return farthestPairInXDirection.first;
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
        imu = hardwareMap.get(BNO055IMU.class,"imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
//        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private enum MineralType {
        Gold, Silver, None
    }

    private void moveByEncoder(int encoderDistance, double x, double y){
        DriveTrain.mecanum(baseMotorArray,0,0,0,true);
        double power = clip(Math.sqrt((x * x) + (y * y)),-1.0,1.0);
//        double radianAngle = 0;
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
        DriveTrain.mecanum(baseMotorArray,0,1,0,true);
        imuWallImpactDetector.setImpact(false);
        safeSleep(300);
        int encoderChange = 1000;
        int previousEncoderPosition = ((DcMotor)baseMotorArray.get(0)).getCurrentPosition();
        int numTimes_looped = 0;
        while ((Math.abs(encoderChange) > 5 || numTimes_looped < 3)&&opModeIsActive() && numTimes_looped < 200 && !imuWallImpactDetector.isImpact()) {
            numTimes_looped++;
            safeSleep(30);
            encoderChange = ((DcMotor) baseMotorArray.get(0)).getCurrentPosition() - previousEncoderPosition;
            previousEncoderPosition = ((DcMotor) baseMotorArray.get(0)).getCurrentPosition();
        }
        safeSleep(100);
        DriveTrain.mecanum(baseMotorArray,0,0,0,true);
    }
    private void safeSleep(int sleepTime){
        try {
            Thread.sleep(sleepTime);
        }catch (InterruptedException e){
            System.out.println(e);
        }
    }
    void turnToAngle(double desiredAngle){
        // angle in degrees. clock-wise is positive
        // angle where the lander position is 45 degrees counterclockwise
        double angleToTurn = imu.getAngularOrientation().thirdAngle+desiredAngle+45;
        DriveTrain.turn(baseMotorArray,angleToTurn,wheelWidthBetweenWheels, wheelHeighBetweenWheels);
    }
}
