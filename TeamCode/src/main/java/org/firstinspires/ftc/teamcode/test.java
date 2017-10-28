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

@Autonomous(name = "Vashon 5961 test", group = "Vashon 5961")
public class test extends LinearOpMode {
    private ArrayList baseMotorArray = new ArrayList();
    private VuforiaLocalizer vuforia;
    OpenGLMatrix lastLocation = null;
    private static final String TAG = "Vuforia Navigation Sample";
    //    ColorSensor colorSensor;
    double wheelCircumference = 100.0 * Math.PI; // circumference in mm
    double ticksPerRotation = 1125.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // get wheels from config
        baseMotorArray.add(hardwareMap.dcMotor.get("front Left"));
        baseMotorArray.add(hardwareMap.dcMotor.get("front Right"));
        baseMotorArray.add(hardwareMap.dcMotor.get("back Left"));
        baseMotorArray.add(hardwareMap.dcMotor.get("back Right"));
        ((DcMotor) baseMotorArray.get(1)).setDirection(DcMotor.Direction.REVERSE);
        ((DcMotor) baseMotorArray.get(3)).setDirection(DcMotor.Direction.REVERSE);

        DriveTrain.mecanum(baseMotorArray,-1.0,0.0,0.0);
        sleep(1000);
        DriveTrain.mecanum(baseMotorArray,0.0,-1.0,0.0);
        sleep(1000);
        DriveTrain.mecanum(baseMotorArray,1.0,0.0,0.0);
        sleep(1000);
        DriveTrain.mecanum(baseMotorArray,0.0,1.0,0.0);
        sleep(1000);
        DriveTrain.mecanum(baseMotorArray,0.0,1.0,90.0);
        sleep(500);
        DriveTrain.mecanum(baseMotorArray,0.0,-1.0,-90.0);
        sleep(500);
        stop();

    }
}