package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

/**
 * Created by FTC on 10/28/2017.
 */

public abstract class JustDriveAuto extends LinearOpMode{


    private Servo leftServo;
    private Servo rightServo;
    private DcMotor lift;
    private double motorSpeedMultiplier = 1.0;
    private ArrayList baseMotorArray;
    double wheelCircumference = 100.0 * Math.PI;
    double ticksPerRotation = 1125.0;

    private final boolean isRed;
    private final boolean isCorner;

    public JustDriveAuto(boolean isRed, boolean isCorner) {
        this.isRed = isRed;
        this.isCorner = isCorner;
    }

    @Override
    public void runOpMode() throws InterruptedException{

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
        leftServo.setPosition(0.8);
        rightServo.setPosition(0.5);
        grabGlyph();
        waitForStart();
        moveToGlyph();
        dropGlyph();
        moveAwayFromGlyph();

    }



    private void mecanumDriveForDistance(Double angle, Double power, Double distance){
        Double radians = (angle * Math.PI) / 180.0;
        Double x = Math.cos(radians) * power;
        Double y = Math.sin(radians) * power;
        DriveTrain.mecanum(baseMotorArray, x, y, 0.0);
        Integer startPos = ((DcMotor)baseMotorArray.get(1)).getCurrentPosition();
        while (Math.abs(((DcMotor) baseMotorArray.get(1)).getCurrentPosition()) < ((int)((distance / wheelCircumference) * ticksPerRotation) + startPos)){
            sleep(10);
        }
    }

    void grabGlyph() {

        leftServo.setPosition(0.0);
        rightServo.setPosition(0.0);

    }

    void moveToGlyph() {
        if (isCorner) {
            if (!isRed) {
                DriveTrain.mecanum(baseMotorArray, 0.0, 0.0, 1.0);
                sleep(1800);
            }

            mecanumDriveForDistance(0.0, 1.0, 1250.0);
        }
        else {
            DriveTrain.mecanum(baseMotorArray, 0.0, 1.0, 0.0);
            sleep(1000);
            if (!isRed) {
                DriveTrain.mecanum(baseMotorArray, 0.0, 0.0, 1.0);
                sleep(1800);
            }

            mecanumDriveForDistance(0.0, 1.0, 500.0);
        }
    }

    void dropGlyph() {

        leftServo.setPosition(0.8);
        rightServo.setPosition(0.5);

    }

    private void moveAwayFromGlyph() {
        sleep(300);
        lift.setPower(1);
        sleep(600);
        lift.setPower(0);
        sleep(100);
        DriveTrain.mecanum(baseMotorArray, -1.0, 0.0, 0.0);
        sleep(400);
    }

}
