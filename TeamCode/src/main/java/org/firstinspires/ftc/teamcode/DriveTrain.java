package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


/**
 * Created by Caz on 9/30/2017.
 */

public class DriveTrain {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    public DriveTrain(DcMotor fL,DcMotor fR,DcMotor bL,DcMotor bR) {
        frontLeft = fL;
        frontRight = fR;
        backLeft = bL;
        backRight = bR;
    }
    //    baseMotorArray goes in order: frontLeft, frontRight, backLeft, backRight
    public static void nonMecanum(ArrayList baseMotorArray, double power[]) {
    }

    public void mecanum(double x, double y, double turn){
        double power = Math.sqrt((x * x) + (y * y));
//        double radianAngle = 0;
        double radianAngle = Math.atan2(y, x) - Math.PI / 4;
//
        if (Math.abs(power) + Math.abs(turn) > 1)
        {
            power = power /(Math.abs(power) + Math.abs(turn));
            turn = Math.signum(turn) * (1 - Math.abs(power));
        }

//        double motorPower[] = {
        frontLeft.setPower((Math.cos(radianAngle) * power) + turn);
        frontRight.setPower((Math.cos(radianAngle) * power) - turn);
        backLeft.setPower((Math.sin(radianAngle) * power) + turn);
        backRight.setPower((Math.sin(radianAngle) * power) - turn);
//        };

//        for (int i = 0; i < baseMotorArray.size(); i++) {
//            ((DcMotor) baseMotorArray.get(i)).setPower(maxUnit(motorPower[i]));
//        }
    }

    private static double maxUnit(final double input) {
        return input > 1 ? 1 : input < -1 ? -1 : input;
    }
}
