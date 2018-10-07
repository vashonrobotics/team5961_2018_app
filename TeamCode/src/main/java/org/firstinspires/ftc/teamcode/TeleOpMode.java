package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

/**
 * Created by FTC on 9/23/2017.
 * updated by Caz
 */

@TeleOp(name = "Vashon 5961 teleop", group = "Vashon 5961")
public class TeleOpMode extends OpMode{
    private Servo leftServo;
    private Servo rightServo;
    private DcMotor lift;
    private double motorSpeedMultiplier = 1.0;
    private ArrayList baseMotorArray;
    private int liftStartPos;
    private int maxLiftDiff;
    private ColorSensor jewelColor;
    private Servo jewelMover;
    private Servo jewelExtender;
//    private DcMotor relicGrabberExtender;
//    private Servo relicGrabber;
//    private Servo relicLifter;
    private Boolean setMode = false;
    private double relicLifterPos = 0.0;
    private int maxRelicArmPos = 5000; // needs to be tested

    @Override
    public void init() {
        // base motor init
        baseMotorArray = new ArrayList();
        baseMotorArray.add(hardwareMap.dcMotor.get("motorLF"));
        baseMotorArray.add(hardwareMap.dcMotor.get("motorRF"));
        baseMotorArray.add(hardwareMap.dcMotor.get("motorLB"));
        baseMotorArray.add(hardwareMap.dcMotor.get("motorRB"));
        ((DcMotor)baseMotorArray.get(1)).setDirection(DcMotor.Direction.REVERSE);
        ((DcMotor)baseMotorArray.get(3)).setDirection(DcMotor.Direction.REVERSE);
        // lift init
//        lift = hardwareMap.dcMotor.get("lift");
        // lift grabbers
//        leftServo = hardwareMap.servo.get("left");
//        rightServo = hardwareMap.servo.get("right");
//        leftServo.setDirection(Servo.Direction.REVERSE);
//        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        liftStartPos = lift.getCurrentPosition();
        maxLiftDiff = 2050;

        for(int i = 0; i < 4; i++){
            ((DcMotor)baseMotorArray.get(i)).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
//        jewelColor = hardwareMap.colorSensor.get("jewelColor");
//        jewelMover = hardwareMap.servo.get("jewelMover");
//        jewelExtender = hardwareMap.servo.get("jewelExtender");
//
//        jewelColor.enableLed(true);


//        relicGrabberExtender = hardwareMap.dcMotor.get("relicArm");
//        relicGrabberExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        relicGrabberExtender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        relicGrabber = hardwareMap.servo.get("relicGrabber");
//        relicLifter = hardwareMap.servo.get("relicLifter");



    }

    @Override
    public void loop() {
//        if (gamepad2.a) {
//            jewelExtender.setPosition(1);
//        }else {
//            jewelExtender.setPosition(0.8);
//        }
//        if (gamepad2.b){
//            jewelMover.setPosition(1);
//        }else {
//            jewelMover.setPosition(0.8);
//        }

//        float[] colorInHSV = {0f, 0f, 0f};
//        Color.RGBToHSV(jewelColor.red(), jewelColor.green(), jewelColor.blue(), colorInHSV);
//        telemetry.addData("HSV: ", colorInHSV);
//        telemetry.addData("H: ", colorInHSV[0]);

        if (gamepad1.right_trigger >= 0.5) {
            motorSpeedMultiplier = 0.5;
        }else {
            motorSpeedMultiplier = 1.0;
        }
//        telemetry.addData("motorModiFire: ",motorSpeedMultiplier);

        if (gamepad1.left_trigger >= 0.5) {
            DriveTrain.mecanum(baseMotorArray, ((double) gamepad1.left_stick_y) * motorSpeedMultiplier,
                    ((double) gamepad1.left_stick_x) * motorSpeedMultiplier,
                    ((double) gamepad1.right_stick_x) * motorSpeedMultiplier, true);
        }else {
            DriveTrain.mecanum(baseMotorArray, ((double) gamepad1.left_stick_x) * motorSpeedMultiplier,
                    (-(double) gamepad1.left_stick_y) * motorSpeedMultiplier,
                    ((double) gamepad1.right_stick_x) * motorSpeedMultiplier, true);
        }


            //  up is negative
        if ((-gamepad2.left_stick_y > 0 && (maxLiftDiff <= Math.abs(lift.getCurrentPosition()-liftStartPos)))
                || (-gamepad2.left_stick_y < 0 && (Math.abs(lift.getCurrentPosition()) <= 10))) {
//            lift.setPower(0);

        }else {
//            lift.setPower(gamepad2.left_stick_y*3/4);
        }
//        if((gamepad2.right_stick_x < 0 && (maxRelicArmPos <= Math.abs(relicGrabberExtender.getCurrentPosition())))) {
//            relicGrabberExtender.setPower(0);
//        }else {
//            relicGrabberExtender.setPower(gamepad2.right_stick_x);
//        }
//        telemetry.addData("relicArmPos: ", relicGrabberExtender.getCurrentPosition());
//        if(gamepad2.left_trigger >= 0.5){
//            relicGrabber.setPosition(1);
//        }else {
//            relicGrabber.setPosition(0);
//        }
//
//        relicLifterPos += gamepad2.right_stick_y/10;
//        relicLifter.setPosition(relicLifterPos);

//        telemetry.addData("lift pos: ", lift.getCurrentPosition());
//        telemetry.addData("max lift pos: ", maxLiftPos);
//        telemetry.addData("lift start pos: " , liftStartPos);


//        telemetry.addData("stick pos: ", gamepad2.left_stick_y);
//        telemetry.addData("up bound stopped: ", (-gamepad2.left_stick_y > 0 && (Math.abs(maxLiftPos) <= Math.abs(lift.getCurrentPosition()))));
//        telemetry.addData("low bound stopped: ", (-gamepad2.left_stick_y < 0 && (Math.abs(liftStartPos) >= Math.abs(lift.getCurrentPosition()))));
//        telemetry.update();

        if(gamepad2.right_trigger >= .5){

//            leftServo.setPosition(0.7);
//            rightServo.setPosition(0.7);

        }else{

//            leftServo.setPosition(1.0);
//            rightServo.setPosition(0.9);

        }
//        telemetry.update();
    }
}
