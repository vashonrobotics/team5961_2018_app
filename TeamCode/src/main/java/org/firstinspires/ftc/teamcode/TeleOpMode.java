package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

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


    @Override
    public void init() {
        frontLeft = hardwareMap.dcMotor.get("front Left");
        frontRight = hardwareMap.dcMotor.get("front Right");
        backLeft = hardwareMap.dcMotor.get("back Left");
        backRight = hardwareMap.dcMotor.get("back Right");
    }

    @Override
    public void loop() {
        frontLeft.setPower(gamepad1.right_stick_y-gamepad1.right_stick_x);
        frontRight.setPower(gamepad1.right_stick_y+gamepad1.right_stick_x);
        backLeft.setPower(gamepad1.right_stick_y-gamepad1.right_stick_x);
        backRight.setPower(gamepad1.right_stick_y+gamepad1.right_stick_x);
    }
}
