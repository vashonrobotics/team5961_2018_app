package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

/**
 * Created by FTC on 9/23/2017.
 * updated by Caz
 */

@TeleOp(name = "Vashon 5961 teleop", group = "Vashon 5961")
//@Disabled
public class TeleOpMode extends OpMode{
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private Servo left;
    private Servo right;
    private DcMotor lift;
    private int closeAngle=90;
    DriveTrain driveTrain;
    private ArrayList baseMotorArray;
    @Override
    public void init() {
        // base motor init
        baseMotorArray = new ArrayList();
        baseMotorArray.add(hardwareMap.dcMotor.get("front Left"));
        baseMotorArray.add(hardwareMap.dcMotor.get("front Right"));
        baseMotorArray.add(hardwareMap.dcMotor.get("back Left"));
        baseMotorArray.add(hardwareMap.dcMotor.get("back Right"));
        ((DcMotor)baseMotorArray.get(0)).setDirection(DcMotor.Direction.REVERSE);
        ((DcMotor)baseMotorArray.get(2)).setDirection(DcMotor.Direction.REVERSE);
        // lift motor init
        lift = hardwareMap.dcMotor.get("lift");
        left = hardwareMap.servo.get("left");
        right = hardwareMap.servo.get("right");
        left.setPosition(0);
        right.setPosition(0);
        driveTrain = new DriveTrain(hardwareMap.dcMotor.get("front Left"),hardwareMap.dcMotor.get("front Right"),hardwareMap.dcMotor.get("back Left"),hardwareMap.dcMotor.get("back Right"));;

    }

    @Override
    public void loop() {

        driveTrain.mecanum( (double) -gamepad1.left_stick_x, (double) gamepad1.left_stick_y, (double)gamepad1.right_stick_x);
        lift.setPower(gamepad2.left_stick_y);
        if(gamepad2.right_bumper){

            left.setPosition(-closeAngle);
            right.setPosition(closeAngle);

        }else{

            left.setPosition(0);
            right.setPosition(0);

        }

    }
}
