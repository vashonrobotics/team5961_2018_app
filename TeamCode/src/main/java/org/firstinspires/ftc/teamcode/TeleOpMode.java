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
    private DcMotor lift;
    private double motorSpeedMultiplier = 1.0;
    private ArrayList baseMotorArray;
    private int liftTargetPos = 0;
    private DcMotor collectorExtender;
    private DcMotor collectorRotator;
    private Servo collectorGrabber;
    private Servo collectorGrabberRotator;
    private Servo markerDropper;
    private Boolean setMode = false;
    private double relicLifterPos = 0.0;
    private int maxRelicArmPos = 5000; // needs to be tested
//    boolean pressedA = false;

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
        lift = hardwareMap.dcMotor.get("lift");

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        markerDropper = hardwareMap.servo.get("dropper");
//        markerDropper.scaleRange(0,1);
//        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        lift.setTargetPosition(0);
//        lift.setPower(1);
//        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // collector init
//        collectorRotator = hardwareMap.dcMotor.get("colRot");
//        collectorExtender = hardwareMap.dcMotor.get("colExt");
//        collectorGrabber = hardwareMap.servo.get("colGrab");
//        collectorGrabberRotator = hardwareMap.servo.get("colGrabRot");
//        collectorGrabberRotator.setPosition(0);
//        collectorGrabber.setPosition(0);
        for(int i = 0; i < 4; i++){
            ((DcMotor)baseMotorArray.get(i)).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    @Override
    public void loop() {

        telemetry.addData("lift encoder", lift.getCurrentPosition());
//        telemetry.addData("lift Target Pos", liftTargetPos);
//        if (gamepad2.right_stick_x > 0.1 || gamepad2.right_stick_x < -0.1) {
//            liftTargetPos += gamepad2.right_stick_x*2;
//
//            lift.setTargetPosition(liftTargetPos);
//            lift.setPower(1);
//        }

        lift.setPower(gamepad2.right_stick_x);
//        if (gamepad2.a){
//            pressedA = true;
//        }
//        if (gamepad2.b){
//            pressedA = false;
//        }
//        if (pressedA){
//            lift.setPower(-0.1);
//        }
//        if (gamepad2.x){
//            markerDropper.setPosition(1);
//        }else{
//            markerDropper.setPosition(0);
//        }
        if (gamepad1.right_trigger >= 0.5) {
            motorSpeedMultiplier = 0.4;
        }else {
            motorSpeedMultiplier = 1.0;
        }
            DriveTrain.mecanum(baseMotorArray, ((double) gamepad1.left_stick_x) * motorSpeedMultiplier,
                    (-(double) gamepad1.left_stick_y) * motorSpeedMultiplier,
                    ((double) gamepad1.right_stick_x) * motorSpeedMultiplier, true);


            //  up is negative
        // robot lifter stuff
//        lift.setPower(gamepad2.right_stick_x);

        // collector stuff
//        collectorExtender.setPower(gamepad2.left_stick_x);
//        collectorRotator.setPower(gamepad2.left_stick_y/2);
//        if (gamepad2.right_bumper) {
//            collectorGrabberRotator.setPosition(1);
//        }else{
//            collectorGrabberRotator.setPosition(0);
//        }
//        if (gamepad2.left_bumper){
//            collectorGrabber.setPosition(1);
//        }else{
//            collectorGrabber.setPosition(0);
//        }
        telemetry.update();
    }
}
