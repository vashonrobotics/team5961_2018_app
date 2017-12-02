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
    private int maxLiftDiffPos;
    private ColorSensor jewelColor;
//    private int caz = 1;
    private DcMotor relicGrabberExtender;
    private Servo RelicGrabber;
    private Servo RelicLifter;
    private Boolean setMode = false;

    @Override
    public void init() {
        // base motor init
        baseMotorArray = new ArrayList();
        baseMotorArray.add(hardwareMap.dcMotor.get("front Left"));
        baseMotorArray.add(hardwareMap.dcMotor.get("front Right"));
        baseMotorArray.add(hardwareMap.dcMotor.get("back Left"));
        baseMotorArray.add(hardwareMap.dcMotor.get("back Right"));
        ((DcMotor)baseMotorArray.get(1)).setDirection(DcMotor.Direction.REVERSE);
        ((DcMotor)baseMotorArray.get(3)).setDirection(DcMotor.Direction.REVERSE);
        // lift init
        lift = hardwareMap.dcMotor.get("lift");
        // lift grabbers
        leftServo = hardwareMap.servo.get("left");
        rightServo = hardwareMap.servo.get("right");
        leftServo.setDirection(Servo.Direction.REVERSE);
        liftStartPos = lift.getCurrentPosition();
        maxLiftDiffPos = 2050;
        for(int i = 0; i < 4; i++){
            ((DcMotor)baseMotorArray.get(i)).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
//        jewelColor = hardwareMap.colorSensor.get("jewelColor");
//        jewelColor.enableLed(false);


//        relicGrabberExtender = hardwareMap.dcMotor.get("relicArm");
//        RelicGrabber = hardwareMap.servo.get("relicGrabber");
//        RelicLifter = hardwareMap.servo.get("relicLifter");



    }

    @Override
    public void loop() {


//
//        telemetry.addData("R: ", jewelColor.red());
//        telemetry.addData("G: ", jewelColor.green());
//        telemetry.addData("B: ", jewelColor.blue());
//        float[] colorInHSV = {};
//        Color.RGBToHSV(jewelColor.red(), jewelColor.green(), jewelColor.blue(), colorInHSV);
//        telemetry.addData("HSV: ", colorInHSV);
//        telemetry.addData("H: ", colorInHSV[0]);

        if (gamepad1.right_trigger >= 0.5) {
            motorSpeedMultiplier = 0.5;
        }else {
            motorSpeedMultiplier = 1.0;
        }
//        telemetry.addData("motorModiFire: ",motorSpeedMultiplier);
//        telemetry.update();
        if (gamepad1.left_trigger >= 0.5) {
            DriveTrain.mecanum(baseMotorArray, ((double) gamepad1.left_stick_y) * motorSpeedMultiplier,
                    ((double) gamepad1.left_stick_x) * motorSpeedMultiplier,
                    ((double) gamepad1.right_stick_x) * motorSpeedMultiplier);
        }else {
            DriveTrain.mecanum(baseMotorArray, ((double) gamepad1.left_stick_x) * motorSpeedMultiplier,
                    ((double) gamepad1.left_stick_y) * motorSpeedMultiplier,
                    ((double) gamepad1.right_stick_x) * motorSpeedMultiplier);
        }


            //  up is negative
        if ((-gamepad2.left_stick_y > 0 && (maxLiftDiffPos <= Math.abs(lift.getCurrentPosition()-liftStartPos)))) {
            lift.setPower(0);

        }else {
            lift.setPower(gamepad2.left_stick_y*3/4);
        }
//        telemetry.addData("lift pos: ", lift.getCurrentPosition());
//        telemetry.addData("max lift pos: ", maxLiftPos);
//        telemetry.addData("lift start pos: " , liftStartPos);


//        telemetry.addData("stick pos: ", gamepad2.left_stick_y);
//        telemetry.addData("up bound stopped: ", (-gamepad2.left_stick_y > 0 && (Math.abs(maxLiftPos) <= Math.abs(lift.getCurrentPosition()))));
//        telemetry.addData("low bound stopped: ", (-gamepad2.left_stick_y < 0 && (Math.abs(liftStartPos) >= Math.abs(lift.getCurrentPosition()))));
//        telemetry.update();

        if(gamepad2.right_trigger >= .5){

            leftServo.setPosition(0.2);
            rightServo.setPosition(0.2);

        }else{

            leftServo.setPosition(0.9);
            rightServo.setPosition(0.8);

        }

    }
}
