package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;



/**
 * Created by Caz on 9/30/2017.
 */

public class DriveTrain {
//    baseMotorArray goes in order: frontLeft, frontRight, backLeft, backRight
    public static void nonMecanum(ArrayList baseMotorArray, ArrayList power) {
        for (int x = 0; x < baseMotorArray.size(); x++) {
            ((DcMotor) baseMotorArray.get(x)).setPower((double) power.get(x));
        }

    }
    public static void mecanum(ArrayList baseMotorArray, Double angle, Double turn){
        // angle is in degrees
        // positive turn = turn right
        ArrayList powerArray = new ArrayList();
        double radianAngle = angle*Math.PI/180;

        powerArray.add(Math.cos(radianAngle)+turn);// frontLeft
        powerArray.add(-1*Math.cos(radianAngle)-turn);// frontRight
        powerArray.add(-1*Math.cos(radianAngle)+turn);// backLeft
        powerArray.add(Math.cos(radianAngle)-turn);// backRight
        nonMecanum(baseMotorArray, powerArray);
    }
}
