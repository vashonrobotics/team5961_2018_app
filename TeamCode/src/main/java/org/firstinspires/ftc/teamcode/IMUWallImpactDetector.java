package org.firstinspires.ftc.teamcode;

import android.support.annotation.NonNull;
import android.support.annotation.Nullable;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class IMUWallImpactDetector implements BNO055IMU.AccelerationIntegrator {
    private static final double THRESHOLD = 4;
    private final BNO055IMU.AccelerationIntegrator realIntegrator;
    private final Telemetry telemetry;
    private boolean impact = false;
    private double maxAcceleration = 0;
    private double timeReset;

    private final Object[] lock = new Object[0];

    public IMUWallImpactDetector(Telemetry telemetry, BNO055IMU.AccelerationIntegrator realIntegrator) {
        this.realIntegrator = realIntegrator;
        this.telemetry = telemetry;
    }

    @Override
    public void initialize(@NonNull BNO055IMU.Parameters parameters, @Nullable Position initialPosition, @Nullable Velocity initialVelocity) {
        realIntegrator.initialize(parameters, initialPosition, initialVelocity);

    }

    @Override
    public Position getPosition() {
        return realIntegrator.getPosition();
    }

    @Override
    public Velocity getVelocity() {
        return realIntegrator.getVelocity();
    }

    @Override
    public Acceleration getAcceleration() {
        return realIntegrator.getAcceleration();
    }

    @Override
    public void update(Acceleration linearAcceleration) {
        final double magnitude = Math.sqrt(Math.pow(linearAcceleration.xAccel, 2) +
                Math.pow(linearAcceleration.yAccel, 2)+Math.pow(linearAcceleration.zAccel,2));


        maxAcceleration = Math.max(maxAcceleration, magnitude);
        if (magnitude > THRESHOLD && System.currentTimeMillis()-timeReset > 500) {
            telemetry.addData("Impact Detected,", "Acceleration Magnitude: %f", magnitude);
            telemetry.update();
            setImpact(true);

        }else{
            telemetry.addData("active ",(System.currentTimeMillis()-timeReset > 300));
//            telemetry.update();
//            telemetry.addData("Max Acceleration", "Max Acceleration %f", maxAcceleration);
            telemetry.addData("acceleration",magnitude);
            telemetry.update();
        }
        realIntegrator.update(linearAcceleration);
    }

    public boolean isImpact() {
        synchronized (lock) {
            return impact;
        }
    }

    public void setImpact(boolean impact) {
        synchronized (lock) {
            this.impact = impact;
            if (!impact) {
                timeReset = System.currentTimeMillis();
            }
        }
    }
}