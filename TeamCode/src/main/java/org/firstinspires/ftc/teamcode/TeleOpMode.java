package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

import static com.qualcomm.robotcore.util.Range.clip;

/**
 * Created by FTC on 9/23/2017.
 * updated by Caz
 */

@TeleOp(name = "Vashon 5961 teleop", group = "Vashon 5961")
public class TeleOpMode extends OpMode{

    double wheelWidthBetweenWheels = 279.4;//215;
    double wheelHeighBetweenWheels = 257;//340;
    private DcMotor lift;
    private double motorSpeedMultiplier = 1.0;
    private ArrayList baseMotorArray = new ArrayList();
    private int liftTargetPos = 0;
    private DcMotor collectorArmExtender;
    private DcMotor collectorArmRotator;
    private CRServo collector;

    private Servo collectorRotator;
    private Servo markerDropper;
    private Boolean setMode = false;
    private int previousBaseMotorPos = -1;

    private BNO055IMU imu;
//    private CRServo stickyArm;
//    boolean pressedA = false;

    @Override
    public void init() {
        // base motor init
        baseMotorArray.add(hardwareMap.dcMotor.get("motorLF"));
        baseMotorArray.add(hardwareMap.dcMotor.get("motorRF"));
        baseMotorArray.add(hardwareMap.dcMotor.get("motorLB"));
        baseMotorArray.add(hardwareMap.dcMotor.get("motorRB"));
        ((DcMotor)baseMotorArray.get(1)).setDirection(DcMotor.Direction.REVERSE);
        ((DcMotor)baseMotorArray.get(3)).setDirection(DcMotor.Direction.REVERSE);
//
//        // lift init
        lift = hardwareMap.dcMotor.get("lift");

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        stickyArm = hardwareMap.crservo.get("stickyArm");

//
//        markerDropper = hardwareMap.servo.get("dropper");

        // collector init
        collectorArmRotator = hardwareMap.dcMotor.get("rotate");
        collectorArmRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collectorArmExtender = hardwareMap.dcMotor.get("extend");
        collectorArmExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collector = hardwareMap.crservo.get("collector");
        collectorRotator = hardwareMap.servo.get("assistant");
        imu = hardwareMap.get(BNO055IMU.class,"imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
        for(int i = 0; i < 4; i++){
            ((DcMotor)baseMotorArray.get(i)).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        collectorRotator.scaleRange(0.2,1);
    }

    @Override
    public void loop() {
        for (int i = 0; i < 4; i++) {
            telemetry.addData("encoder "+Integer.valueOf(i),((DcMotor) baseMotorArray.get(i)).getCurrentPosition());
        }
//        telemetry.addData("accel calibrated", imu.isAccelerometerCalibrated());
//        telemetry.addData("gyro calibrated", imu.isGyroCalibrated());
//        telemetry.addData("accel" + imu.getAcceleration().xAccel + ", "+ imu.getAcceleration().yAccel, imu.getAcceleration().zAccel);
//        telemetry.addData("rotation 1: ", imu.getAngularOrientation().firstAngle);
//        telemetry.addData("rotation 2: ", imu.getAngularOrientation().secondAngle);
//        telemetry.addData("rotation 3: ", imu.getAngularOrientation().thirdAngle);
//        telemetry.addData("mag field x"+imu.getMagneticFieldStrength().x+",y "+imu.getMagneticFieldStrength().y+", z", imu.getMagneticFieldStrength().z);
        telemetry.addData("collector power:",collector.getPower());
// telemetry.addData("arm power",stickyArm.getPower());
//        if (gamepad1.right_stick_x == 0 && gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0) {
////            ((DcMotor) baseMotorArray.get(0)).getCurrentPosition();
//            if (previousBaseMotorPos != -1) {
//                if(previousBaseMotorPos != ((DcMotor) baseMotorArray.get(0)).getCurrentPosition()){
//                    telemetry.addData("change in encoder values", ((DcMotor) baseMotorArray.get(0)).getCurrentPosition() - previousBaseMotorPos);
//                }
//            }else{
//                previousBaseMotorPos = ((DcMotor) baseMotorArray.get(0)).getCurrentPosition();
//            }
//        }
//        if(gamepad1.x){
//            previousBaseMotorPos = -1;
//        }
////        for (int i = 0; i < baseMotorArray.size(); i++) {
////            DcMotor motor = ((DcMotor) baseMotorArray.get(i));
////            telemetry.addData("motor " + i, motor.getCurrentPosition());
////        }
////        telemetry.addData("lift encoder", lift.getCurrentPosition());
////        telemetry.addData("lift Target Pos", liftTargetPos);
////        if (gamepad2.right_stick_x > 0.1 || gamepad2.right_stick_x < -0.1) {
////            liftTargetPos += gamepad2.right_stick_x*2;
////
////            lift.setTargetPosition(liftTargetPos);
////            lift.setPower(1);
////        }
//
        lift.setPower(gamepad2.right_stick_x);
//
////        stickyArm.setPower(clip(gamepad2.left_stick_x/2-.6,-1,1));
//
//
        if (gamepad1.y){
            fixBump(0);
        }
        if (gamepad1.b){
            DriveTrain.turn(baseMotorArray,90,wheelWidthBetweenWheels,wheelHeighBetweenWheels);
        }

        if (gamepad1.right_trigger >= 0.5) {
            motorSpeedMultiplier = 1;
        }else {
            motorSpeedMultiplier = 0.4;
        }
        if (gamepad1.right_bumper) {
            goStraight(((double) gamepad1.left_stick_x) * motorSpeedMultiplier, ((double) -gamepad1.left_stick_y) * motorSpeedMultiplier);
        }else {
            DriveTrain.mecanum(baseMotorArray, ((double) gamepad1.left_stick_x) * motorSpeedMultiplier,
                    (-(double) gamepad1.left_stick_y) * motorSpeedMultiplier,
                    ((double) gamepad1.right_stick_x) * motorSpeedMultiplier, true);
        }
        if (gamepad1.x){
            DriveTrain.mecanum(baseMotorArray,0,1,0,true);
        }
////
////            //  up is negative
//        // collector stuff

        collectorArmExtender.setPower(gamepad2.left_stick_x);
////        collectorArmRotator.setPower(Math.sqrt(gamepad2.left_stick_y));
        collectorArmRotator.setPower(Math.signum(gamepad2.left_stick_y)*Math.pow(gamepad2.left_stick_y,2));
        collector.setPower(clip((gamepad2.right_trigger-gamepad2.left_trigger)*0.8,-0.8,0.8));
        if (gamepad2.left_bumper){
            collectorRotator.setPosition(collectorRotator.getPosition()+0.01);
        }else if (gamepad2.right_bumper){
            collectorRotator.setPosition(collectorRotator.getPosition()-0.01);
        }
        if(gamepad2.y) {
            collectorRotator.setPosition(1);
        }



        if(gamepad2.x) {
            collectorRotator.setPosition(0);
        }
//        collector.setPower(gamepad1.left_trigger*2-1);
//        if (gamepad2.right_bumper) {
//            collectorGrabber.setPosition(1);
//        }
//        if (gamepad2.left_bumper){
//            collectorGrabber.setPosition(0.3);
//        }
//        collectorGrabberRotator.setPosition(clip(collectorGrabberRotator.getPosition()+gamepad2.right_trigger/16-gamepad2.left_trigger/16,0,1));
        telemetry.update();
    }
    private void fixBump(double desiredAngle) {
        // angle in degrees. clock-wise is positive
        double angleToTurn = imu.getAngularOrientation().firstAngle + desiredAngle;
        DriveTrain.turn(baseMotorArray, angleToTurn, wheelWidthBetweenWheels, wheelHeighBetweenWheels);
    }
    private void goStraight(double x,double y){
        double turn;
        if (Math.abs(imu.getAngularOrientation().firstAngle) > 1) {
            turn = Math.signum(imu.getAngularOrientation().firstAngle)*Math.log(Math.abs(imu.getAngularOrientation().firstAngle)) / 10;
        }else {
            turn = 0;
        }
        DriveTrain.mecanum(baseMotorArray,x,y,turn, true);
    }
}
