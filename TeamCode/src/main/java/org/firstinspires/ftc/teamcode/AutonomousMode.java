package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

/**
 * Created by FTC on 9/23/2017.
 * updated by Caz
 */
// error:"can't find "back left""
@Autonomous(name = "Vashon 5961 Autonomous", group = "Vashon 5961")
public class AutonomousMode extends LinearOpMode {
    private ArrayList baseMotorArray;
    @Override
    public void runOpMode() throws InterruptedException {
        double wheelCircumference = 100.0*Math.PI; // circumference in mm
        baseMotorArray = new ArrayList();
        // get wheels from config
        baseMotorArray.add(hardwareMap.dcMotor.get("front Left"));
        baseMotorArray.add(hardwareMap.dcMotor.get("front Right"));
        baseMotorArray.add(hardwareMap.dcMotor.get("back Left"));
        baseMotorArray.add(hardwareMap.dcMotor.get("back Right"));
        ((DcMotor)baseMotorArray.get(1)).setDirection(DcMotor.Direction.REVERSE);
        ((DcMotor)baseMotorArray.get(3)).setDirection(DcMotor.Direction.REVERSE);
        DriveTrain.mecanum(baseMotorArray, 60.0, 0.0);

        sleep(1000);
//        while (((DcMotor)baseMotorArray.get(0)).getCurrentPosition() < 700.0/wheelCircumference){
//            sleep(10);
//        }
        DriveTrain.nonMecanum(baseMotorArray, new double[]{0.0, 0.0, 0.0, 0.0});
        // Read pictograph



    }

}
