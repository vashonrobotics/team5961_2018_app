package org.firstinspires.ftc.teamcode;

import android.graphics.Camera;
import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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

@Autonomous(group = "5961", name = "Autonomous Blue")
public class AutonomousBlue extends LinearOpMode {
    ArrayList baseMotorArray = new ArrayList();
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

    @Override
    public void runOpMode() {
        initalizeRobot();
        waitForStart();
        telemetry.addLine("started");
        telemetry.update();
        setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double[] goldAtPos1 = lookForGold();
        // if gold is straight ahead
        if (goldAtPos1[0] >= 0 && goldAtPos1[1] >= 0 && goldAtPos1[2] >= 0){
//            DriveTrain.mecanum(baseMotorArray, 0, 1, 0, true);
            DriveTrain.mecanum(baseMotorArray, 1, 0, 0, true);
            sleep(3000);
            DriveTrain.mecanum(baseMotorArray, 0,0,0,true);
        }else {

//            DriveTrain.turn(baseMotorArray, 45, 5, 5);
//            sleep(500);
            DriveTrain.mecanum(baseMotorArray, 0,0,1,true);
            sleep(350);
            DriveTrain.mecanum(baseMotorArray, 0,0,0,true);
            setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            double[] goldAtPos2 = lookForGold();
            // if gold is to the right
            if (goldAtPos2[0] >= 0 && goldAtPos2[1] >= 0 && goldAtPos2[2] >=0){
//                DriveTrain.mecanum(baseMotorArray, 0, 1, 0, true);
                DriveTrain.mecanum(baseMotorArray, 1,0, 0,true);
                sleep(2000);
                // supposed to go up and right
                DriveTrain.mecanum(baseMotorArray, 1,-1,0,true);
                sleep(3000);
            }else {
                DriveTrain.mecanum(baseMotorArray, 0,0,-1,true);
                sleep(600);
                DriveTrain.mecanum(baseMotorArray, 0,0,0,true);
//                DriveTrain.turn(baseMotorArray, -90, 5, 5);
//                sleep(500);
                setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                DriveTrain.mecanum(baseMotorArray, 0, 1, 0, true);
                DriveTrain.mecanum(baseMotorArray, 1,0, 0,true);
                sleep(2000);
                // supposed to go up and right
//                DriveTrain.mecanum(baseMotorArray, -1,1,0,true);
                DriveTrain.mecanum(baseMotorArray, 1,1, 0,true);
                sleep(3000);
//                double[] goldAtPos3 = lookForGold();
//                if (goldAtPos3[0] < 0 && goldAtPos3[1] < 0 && goldAtPos3[2] < 0 && NUM_TIME_RESAMPLED < 2) {
//                    NUM_TIME_RESAMPLED++;
//                    sample();
//                }else {

//                }
            }
        }

////        sample();
//        // stuff to check gold detection with robot
//        setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        double[] goldAtPos1 = lookForGold();
//        DriveTrain.mecanum(baseMotorArray, 0,0,1,true);
////        DriveTrain.turn(baseMotorArray, 45, 5, 5);
//        sleep(400);
//        DriveTrain.mecanum(baseMotorArray, 0,0,0,true);
//        setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        double[] goldAtPos2 = lookForGold();
////        DriveTrain.turn(baseMotorArray, -90, 5, 5);
//        DriveTrain.mecanum(baseMotorArray, 0,0,-1,true);
//        sleep(700);
//        DriveTrain.mecanum(baseMotorArray, 0,0,0,true);
//        setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        double[] goldAtPos3 = lookForGold();
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



    private void setMotorRunMode(DcMotor.RunMode runMode) {
        for (Object baseMotorObject: baseMotorArray){
            DcMotor baseMotor = (DcMotor) baseMotorObject;
            baseMotor.setMode(runMode);
        }
    }

    private double[] lookForGold(){
        FtcRobotControllerActivity.shouldProcessImage = true;
        while (FtcRobotControllerActivity.shouldProcessImage) { // should process image is turned to false after the image is processed
            sleep(5);
        }
        ArrayList<double[]> frameOneCandidates = goldDetector.getCandidatesData();
        // when conected to a robot move sideways or turn
        DriveTrain.mecanum(baseMotorArray, 0, -1, 0, true);
        sleep(100);
        DriveTrain.mecanum(baseMotorArray, 0, 0, 0, true);
        sleep(200);
        FtcRobotControllerActivity.shouldProcessImage = true;
        while (FtcRobotControllerActivity.shouldProcessImage) { // should process image is turned to false after the image is processed
            sleep(5);
        }
        ArrayList<double[]> frameTwoCandidates = goldDetector.getCandidatesData();
        ArrayList<Pair<double[], double[]>> pairs = findPairs(frameOneCandidates, frameTwoCandidates);
        ArrayList<Pair<double[], double[]>> acceptablePairs = new ArrayList<>();
        for (Pair<double[], double[]> pair: pairs){
            if (pair.first[2] > 700 && pair.second[2] > 700 && pair.first[2] < 5000 && pair.second[2] < 5000 &&
                    isAboutEqual(pair.first[2],pair.second[2], 3000) &&
                    isAboutEqual(pair.first[1], pair.second[1], 10)) {
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
            telemetry.addData("gold At" + farthestPairInXDirection.first[1] +" " +farthestPairInXDirection.second[1], farthestPairInXDirection.first[2]);
            telemetry.update();
            return farthestPairInXDirection.second;
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
    }
    private enum MineralType {
        Gold, Silver, None;
    }
    private Pair<MineralType, double[]> isThereASilverOrAGoldOrNeitherAndCenterOnIt(){
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
        ArrayList<Double> silverXs = new ArrayList<>();
        ArrayList<Double> silverYs = new ArrayList<>();
        ArrayList<Double> silverSizes = new ArrayList<>();
        for (int i = 0; i < NUM_FRAMES_CONSIDERED; i++){
            ArrayList<double[]> goldDetectorCandidatesData = goldDetector.getCandidatesData();
            double[] goldInfo;
            try {
                goldInfo = goldDetectorCandidatesData.get(goldDetectorCandidatesData.size() - 1);
            }catch (ArrayIndexOutOfBoundsException e){
                telemetry.addLine("no gold found");
                goldInfo = new double[]{0.0,0.0,0.0};
            }
            goldXs.add(goldInfo[0]);
            goldYs.add(goldInfo[1]);
            goldSizes.add(goldInfo[2]);
            ArrayList<double[]> silverDetectorCandidatesData = silverDetector.getCandidatesData();
            double[] silverInfo;
            try {
                silverInfo = goldDetectorCandidatesData.get(goldDetectorCandidatesData.size()-1);
            }catch (ArrayIndexOutOfBoundsException e){
                telemetry.addLine("no silver found");
                silverInfo = new double[]{0.0,0.0,0.0};
            }

            silverXs.add(silverInfo[0]);
            silverYs.add(silverInfo[1]);
            silverSizes.add(silverInfo[2]);
            if (goldInfo[2]<0.1){ //&& silverInfo[2]<0.1){
                numNotFound++;
            }
        }
        Collections.sort(goldXs);
        Collections.sort(goldYs);
        Collections.sort(goldSizes);
        double goldX = (goldXs.get((int)((goldXs.size()-1)/2)) + goldXs.get((int)(goldXs.size()/2)))/2;
        double goldY = (goldYs.get((int)((goldYs.size()-1)/2)) + goldYs.get((int)(goldYs.size()/2)))/2;
        double goldSize = (goldSizes.get((int)((goldSizes.size()-1)/2)) + goldSizes.get((int)(goldSizes.size()/2)))/2;
//        Collections.sort(silverXs);
//        Collections.sort(silverYs);
//        Collections.sort(silverSizes);
//        double silverX = (silverXs.get((int)((silverXs.size()-1)/2)) + silverXs.get((int)(silverXs.size()/2)))/2;
//        double silverY = (silverYs.get((int)((silverYs.size()-1)/2)) + silverYs.get((int)(silverYs.size()/2)))/2;
//        double silverSize = (silverSizes.get((int)((silverSizes.size()-1)/2)) + silverSizes.get((int)(silverSizes.size()/2)))/2;
        Collections.sort(possibleSilversData, new Comparator<double[]>() {
            @Override
            public int compare(double[] lhs, double[] rhs) {
                return Double.compare(lhs[2],rhs[2]);
            }
        });
        if (numNotFound > 2) {
            return new Pair<>(MineralType.None, new double[]{});
        }else {
            return new Pair<>(MineralType.Gold, new double[]{goldX, goldY, goldSize});
        }
    }

//    private double[] getPos(){
////        /*
////         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
////         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
////         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
////         */
//////        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
////        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
////
////        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
////
////        parameters.vuforiaLicenseKey = VUFORIA_KEY ;
////        parameters.cameraDirection   = CAMERA_CHOICE;
////
////        //  Instantiate the Vuforia engine
////        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
//
//        // Load the data sets that for the trackable objects. These particular data
//        // sets are stored in the 'assets' part of our application.
////        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
////        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
////        blueRover.setName("Blue-Rover");
////        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
////        redFootprint.setName("Red-Footprint");
////        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
////        frontCraters.setName("Front-Craters");
////        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
////        backSpace.setName("Back-Space");
//
//        // For convenience, gather together all the trackable objects in one easily-iterable collection */
////        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
////        allTrackables.addAll(targetsRoverRuckus);
//
//        /**
//         * In order for localization to work, we need to tell the system where each target is on the field, and
//         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
//         * Transformation matrices are a central, important concept in the math here involved in localization.
//         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
//         * for detailed information. Commonly, you'll encounter transformation matrices as instances
//         * of the {@link OpenGLMatrix} class.
//         *
//         * If you are standing in the Red Alliance Station looking towards the center of the field,
//         *     - The X axis runs from your left to the right. (positive from the center to the right)
//         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
//         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
//         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
//         *
//         * This Rover Ruckus sample places a specific target in the middle of each perimeter wall.
//         *
//         * Before being transformed, each target image is conceptually located at the origin of the field's
//         *  coordinate system (the center of the field), facing up.
//         */
//
//        /**
//         * To place the BlueRover target in the middle of the blue perimeter wall:
//         * - First we rotate it 90 around the field's X axis to flip it upright.
//         * - Then, we translate it along the Y axis to the blue perimeter wall.
//         */
//        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
//                .translation(0, mmFTCFieldWidth, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
//        blueRover.setLocation(blueRoverLocationOnField);
//
//        /**
//         * To place the RedFootprint target in the middle of the red perimeter wall:
//         * - First we rotate it 90 around the field's X axis to flip it upright.
//         * - Second, we rotate it 180 around the field's Z axis so the image is flat against the red perimeter wall
//         *   and facing inwards to the center of the field.
//         * - Then, we translate it along the negative Y axis to the red perimeter wall.
//         */
//        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
//                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
//        redFootprint.setLocation(redFootprintLocationOnField);
//
//        /**
//         * To place the FrontCraters target in the middle of the front perimeter wall:
//         * - First we rotate it 90 around the field's X axis to flip it upright.
//         * - Second, we rotate it 90 around the field's Z axis so the image is flat against the front wall
//         *   and facing inwards to the center of the field.
//         * - Then, we translate it along the negative X axis to the front perimeter wall.
//         */
//        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
//                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
//        frontCraters.setLocation(frontCratersLocationOnField);
//
//        /**
//         * To place the BackSpace target in the middle of the back perimeter wall:
//         * - First we rotate it 90 around the field's X axis to flip it upright.
//         * - Second, we rotate it -90 around the field's Z axis so the image is flat against the back wall
//         *   and facing inwards to the center of the field.
//         * - Then, we translate it along the X axis to the back perimeter wall.
//         */
//        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
//                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
//        backSpace.setLocation(backSpaceLocationOnField);
//
//        /**
//         * Create a transformation matrix describing where the phone is on the robot.
//         *
//         * The coordinate frame for the robot looks the same as the field.
//         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
//         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
//         *
//         * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
//         * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
//         * camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.
//         *
//         * If using the rear (High Res) camera:
//         * We need to rotate the camera around it's long axis to bring the rear camera forward.
//         * This requires a negative 90 degree rotation on the Y axis
//         *
//         * If using the Front (Low Res) camera
//         * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
//         * This requires a Positive 90 degree rotation on the Y axis
//         *
//         * Next, translate the camera lens to where it is on the robot.
//         * In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and 200 mm above ground level.
//         */
//
//        final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
//        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
//        final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line
//
//        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
//                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
//                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));
//
//        /**  Let all the trackable listeners know where the phone is.  */
//        for (VuforiaTrackable trackable : allTrackables)
//        {
//            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, CAMERA_CHOICE);
//        }
//
//        /** Start tracking the data sets we care about. */
//        targetsRoverRuckus.activate();
//
//            // check all the trackable target to see which one (if any) is visible.
//            targetVisible = false;
//            for (VuforiaTrackable trackable : allTrackables) {
//                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
//                    telemetry.addData("Visible Target", trackable.getName());
//                    targetVisible = true;
//
//                    // getUpdatedRobotLocation() will return null if no new information is available since
//                    // the last time that call was made, or if the trackable is not currently visible.
//                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
//                    if (robotLocationTransform != null) {
//                        lastLocation = robotLocationTransform;
//                    }
//                    break;
//                }
//            }
//
//            // Provide feedback as to where the robot is located (if we know).
//            if (targetVisible) {
//                // express position (translation) of robot in inches.
//                VectorF translation = lastLocation.getTranslation();
//                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
//                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
//
//                // express the rotation of the robot in degrees.
//                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
//                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
//                telemetry.update();
//                return new double[] {translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch};
//            }
//            else {
//                return new double[] {0.0,0.0,0.0};
//            }
//    }
}
