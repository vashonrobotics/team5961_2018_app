package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


/**
 * Created by Caz on 9/30/2017.
 */

public class DriveTrain {

    //    baseMotorArray goes in order: frontLeft, frontRight, backLeft, backRight
    public static void nonMecanum(ArrayList baseMotorArray, double power[]) {
    }

    public static void mecanum(ArrayList baseMotorArray, Double x, Double y, Double turn){
        double power = Math.sqrt((x * x) + (y * y));
//        double radianAngle = 0;
        double radianAngle = Math.atan2(y, x) - Math.PI / 4;
//
        if (Math.abs(power) + Math.abs(turn) > 1)
        {
            power = power /(Math.abs(power) + Math.abs(turn));
            turn = Math.signum(turn) * (1 - Math.abs(power));
        }

        double motorPower[] = {
                (Math.cos(radianAngle) * power) + turn,
                (Math.cos(radianAngle) * power) - turn,
                (Math.sin(radianAngle) * power) + turn,
                (Math.sin(radianAngle) * power) - turn
        };

        for (int i = 0; i < baseMotorArray.size(); i++) {
            ((DcMotor) baseMotorArray.get(i)).setPower(maxUnit(motorPower[i]));
        }
    }

    private static double maxUnit(final double input) {
        return input > 1 ? 1 : input < -1 ? -1 : input;
    }
}
