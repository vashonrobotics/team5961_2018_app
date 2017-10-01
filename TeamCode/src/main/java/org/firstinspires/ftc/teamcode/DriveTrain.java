package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;



/**
 * Created by Caz on 9/30/2017.
 */

public class DriveTrain {
//    baseMotorArray goes in order: frontLeft, frontRight, backLeft, backRight
    public static void nonMecanum(ArrayList baseMotorArray, double power[]) {
        for (int x = 0; x < baseMotorArray.size(); x++) {

            if (power[x] < -1) {
                power[x] = -1;
            }

            else if (power[x] > 1) {
                power[x] = 1;
            }

            ((DcMotor) baseMotorArray.get(x)).setPower((double) power[x]);
        }

    }
    public static void mecanum(ArrayList baseMotorArray, Double angle, Double turn){
        // angle is in degrees
        // positive turn = turn right
        double radianAngle = angle*Math.PI/180.0;


        double powerArray[] = {Math.cos(radianAngle)+turn, -1*Math.cos(radianAngle)-turn, -1*Math.cos(radianAngle)+turn, Math.cos(radianAngle)-turn};
//
//        float powerArray2[] = {1.0f, 1.0f, 1.0f, 1.0f};




//        powerArray.add(Math.cos(radianAngle)+turn);// frontLeft
//        powerArray.add(-1*Math.cos(radianAngle)-turn);// frontRight
//        powerArray.add(-1*Math.cos(radianAngle)+turn);// backLeft
//        powerArray.add(Math.cos(radianAngle)-turn);// backRight
        nonMecanum(baseMotorArray, powerArray);
    }

}
