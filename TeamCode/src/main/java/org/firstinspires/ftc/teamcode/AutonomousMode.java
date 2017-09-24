package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by FTC on 9/23/2017.
 */

public class AutonomousMode extends LinearOpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.dcMotor.get("front Left");
        frontRight = hardwareMap.dcMotor.get("front Right");
        backLeft = hardwareMap.dcMotor.get("back Left");
        backRight = hardwareMap.dcMotor.get("back Right");



    }
    public void driveTrain(Double power, Integer Time) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);


    }
}
