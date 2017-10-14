package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    private DcMotor backLeft;
    private DcMotor backRight;

    private ArrayList baseMotorArray;
    @Override
    public void init() {
        baseMotorArray = new ArrayList();
        baseMotorArray.add(hardwareMap.dcMotor.get("front Left"));
        baseMotorArray.add(hardwareMap.dcMotor.get("front Right"));
        baseMotorArray.add(hardwareMap.dcMotor.get("back Left"));
        baseMotorArray.add(hardwareMap.dcMotor.get("back Right"));
        ((DcMotor)baseMotorArray.get(1)).setDirection(DcMotor.Direction.REVERSE);
        ((DcMotor)baseMotorArray.get(3)).setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
//        I'm keeping this because I don't know if we are using mecanum wheels for sure
//        double powerArray[] = {};
//        powerArray.add((double)(gamepad1.right_stick_y-gamepad1.right_stick_x));
//        powerArray.add((double)(gamepad1.right_stick_y+gamepad1.right_stick_x));
//        powerArray.add((double)(gamepad1.right_stick_y-gamepad1.right_stick_x));
//        powerArray.add((double)(gamepad1.right_stick_y+gamepad1.right_stick_x));
//        DriveTrain.nonMecanum(baseMotorArray, powerArray);
        DriveTrain.mecanum(baseMotorArray,Math.atan((gamepad1.left_stick_y/gamepad1.left_stick_x))*180/Math.PI, (double) gamepad1.right_stick_x);
    }
}
