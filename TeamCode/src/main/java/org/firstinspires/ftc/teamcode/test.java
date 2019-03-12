package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.ArrayList;

/**
 * Created by FTC on 10/21/2017.
 */

//@Autonomous(name = "Vashon 5961 test", group = "Vashon 5961")
public class test extends LinearOpMode {
    private ArrayList baseMotorArray = new ArrayList();
    private VuforiaLocalizer vuforia;
    OpenGLMatrix lastLocation = null;
    //    ColorSensor colorSensor;
    double wheelCircumference = 100.0 * Math.PI; // circumference in mm
    double ticksPerRotation = 1125.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // get wheels from config
        baseMotorArray.add(hardwareMap.dcMotor.get("motorLF"));
        baseMotorArray.add(hardwareMap.dcMotor.get("motorRF"));
        baseMotorArray.add(hardwareMap.dcMotor.get("motorLB"));
        baseMotorArray.add(hardwareMap.dcMotor.get("motorRB"));
        ((DcMotor) baseMotorArray.get(1)).setDirection(DcMotor.Direction.REVERSE);
        ((DcMotor) baseMotorArray.get(3)).setDirection(DcMotor.Direction.REVERSE);

//        DriveTrain.mecanum(baseMotorArray,-1.0,0.0,0.0, false);
//        sleep(1000);
//        DriveTrain.mecanum(baseMotorArray,0.0,-1.0,0.0, false);
//        sleep(1000);
//        DriveTrain.mecanum(baseMotorArray,1.0,0.0,0.0, false);
//        sleep(1000);
//        DriveTrain.mecanum(baseMotorArray,0.0,1.0,0.0, false);
//        sleep(1000);
//        DriveTrain.mecanum(baseMotorArray,0.0,1.0,90.0, false);
//        sleep(500);
//        DriveTrain.mecanum(baseMotorArray,0.0,-1.0,-90.0, false);
//        sleep(500);
//        DriveTrain.mecanum(baseMotorArray,0.0,0.0,0.0, false);
        // to make sure that the configuration is correct
        for (int i = 0; i < 4; i++) {
            ((DcMotor)baseMotorArray.get(i)).setPower(1);
            sleep(1000);
            ((DcMotor)baseMotorArray.get(i)).setPower(0);
        }
        DriveTrain.turn(baseMotorArray,90, 279.4, 257);
//        requestOpModeStop();

    }
}