package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

/**
 * Created by FTC on 9/23/2017.
 * updated by Caz
 */

@TeleOp(name = "Vashon 5961 teleop", group = "Vashon 5961")
//@Disabled
public class TeleOpMode extends OpMode{
    private Servo leftServo;
    private Servo rightServo;
    private DcMotor lift;
    private double motorSpeedMultiplier = 1.0;
    private ArrayList baseMotorArray;
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
        leftServo.setPosition(0.8);
        rightServo.setPosition(0.5);



    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper) {
            motorSpeedMultiplier = 0.5;
        }else {
            motorSpeedMultiplier = 1.0;
        }
        DriveTrain.mecanum(baseMotorArray, ((double) gamepad1.left_stick_x)*motorSpeedMultiplier,
                ((double) gamepad1.left_stick_y)*motorSpeedMultiplier,
                ((double)gamepad1.right_stick_x)*motorSpeedMultiplier);
        lift.setPower(gamepad2.left_stick_y/2);
        if(gamepad2.right_trigger >= .5){

            leftServo.setPosition(0.0);
            rightServo.setPosition(0.0);

        }else{

            leftServo.setPosition(0.8);
            rightServo.setPosition(0.5);

        }

    }
}
