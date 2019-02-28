package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.ArrayList;

import static com.qualcomm.robotcore.util.Range.clip;

/**
 * Created by FTC on 9/23/2017.
 * updated by Caz
 */

@TeleOp(name = "Vashon 5961 teleop", group = "Vashon 5961")
public class TeleOpMode extends OpMode{

    double wheelWidthBetweenWheels = 395.44;//215;
    double wheelHeighBetweenWheels = 0;//340;
    private DcMotor lift;
    private double motorSpeedMultiplier = 1.0;
    private ArrayList baseMotorArray = new ArrayList();
    private int liftTargetPos = 0;
    private DcMotor collectorArmExtender;
    private DcMotor collectorArmRotator;
    private DcMotor collector;
    private Servo collectorRotator;
    private Servo markerDropper;
    private BNO055IMU imu;
    private TouchSensor collectorRotationLimitSwitch;

    private Boolean setMode = false;
    private int previousBaseMotorPos = -1;

    private double collectorArmAngle = 0;
    private double INITAL_ARM_ANGLE = -10;
    private double relativeCollectorPosition; //used assmuming one is close to parallel with the arm
                            // and collector facing out and zero is 180 clock wise from that
    private double desiredEncoderValueForCollectorArm = 0;
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

        collector = hardwareMap.dcMotor.get("collector");
        collectorRotator = hardwareMap.servo.get("assistant");
        imu = hardwareMap.get(BNO055IMU.class,"imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
        for(int i = 0; i < 4; i++){
            ((DcMotor)baseMotorArray.get(i)).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        collectorRotationLimitSwitch = hardwareMap.touchSensor.get("rotationLimitSwitch");
        collectorRotator.scaleRange(0.2,1);
        relativeCollectorPosition = collectorRotator.getPosition();
    }

    @Override
    public void loop() {
        collectorArmAngle = collectorArmRotator.getCurrentPosition()*288/60 - INITAL_ARM_ANGLE; // 288 ticks per rev times 1/60 of a rotation per degree
        double t1 = System.currentTimeMillis();
        telemetry.addData("servo pos", collectorRotator.getPosition());
        lift.setPower(gamepad2.right_stick_x);
//
////        stickyArm.setPower(clip(gamepad2.left_stick_x/2-.6,-1,1));
//
//
//        if (gamepad1.y){
//            fixBump(0);
//        }
//        if (gamepad1.b){
//            DriveTrain.turn(baseMotorArray,360,wheelWidthBetweenWheels,wheelHeighBetweenWheels);
//        }

        if (gamepad1.right_trigger >= 0.5) {
            motorSpeedMultiplier = 1;
        }else {
            motorSpeedMultiplier = 0.4;
        }
//        if (gamepad1.right_bumper) {
//            goStraight(((double) gamepad1.left_stick_x) * motorSpeedMultiplier, ((double) -gamepad1.left_stick_y) * motorSpeedMultiplier);
//        }else {
        DriveTrain.mecanum(baseMotorArray, ((double) gamepad1.left_stick_x) * motorSpeedMultiplier,
                (-(double) gamepad1.left_stick_y) * motorSpeedMultiplier,
                ((double) gamepad1.right_stick_x) * motorSpeedMultiplier, true);
//        }
//        if (gamepad1.x){
//            DriveTrain.mecanum(baseMotorArray,0,1,0,true);
//        }
////
////            //  up is negative
//        // collector stuff

        collectorArmExtender.setPower(gamepad2.left_stick_x);
////        collectorArmRotator.setPower(Math.sqrt(gamepad2.left_stick_y));
        double armPower = Math.signum(-gamepad2.left_stick_y) * Math.sqrt(Math.abs(gamepad2.left_stick_y));
        if (collectorRotationLimitSwitch.isPressed() && armPower < 0) {
                collectorArmRotator.setPower(-0.1);

        }else {
            if (armPower > 0.1 || armPower < -0.1) { // I'm not sure if there could be a time where arm power is zero but isn't read as zero because it's float
                collectorArmRotator.setPower(-armPower);
                desiredEncoderValueForCollectorArm = collectorArmRotator.getCurrentPosition();
            }else{ // if armpower == 0
                if (desiredEncoderValueForCollectorArm != collectorArmRotator.getCurrentPosition()) {
                    double correctionPower = (desiredEncoderValueForCollectorArm -  collectorArmRotator.getCurrentPosition())/20;
                    if (Math.abs(correctionPower) > 0.2){
                        correctionPower = 0.2*Math.signum(correctionPower);
                        telemetry.addData("had to reduce to ",correctionPower);
                    }
                    collectorArmRotator.setPower(correctionPower);
                }
            }
        }
        collector.setPower(gamepad2.right_trigger-gamepad2.left_trigger);
        if (gamepad2.left_bumper){
            relativeCollectorPosition = collectorRotator.getPosition()+0.01;
        }else if (gamepad2.right_bumper){
            relativeCollectorPosition = collectorRotator.getPosition()-0.01;
        }

        if (collectorArmAngle > 60) {
            collectorRotator.setPosition(clip(-(collectorArmAngle-90)/180,0,1));
        }else if (collectorArmAngle > 10){
//            collectorRotator.setPosition(0); //might be better but might want a higher angle
            collectorRotator.setPosition(relativeCollectorPosition-collectorArmAngle/180);
        }else{
            collectorRotator.setPosition(relativeCollectorPosition);
        }

//     dump off minerals
        if(gamepad2.y) {
            double angle_to_be_parallel_to =  90;
            collectorRotator.setPosition(clip(1-(collectorArmAngle-angle_to_be_parallel_to)/180,0,1));
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
        telemetry.addData("time:", System.currentTimeMillis()-t1);

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
