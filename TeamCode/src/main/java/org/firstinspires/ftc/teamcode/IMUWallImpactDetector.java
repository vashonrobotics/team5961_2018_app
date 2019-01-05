package org.firstinspires.ftc.teamcode;

import android.support.annotation.NonNull;
import android.support.annotation.Nullable;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class IMUWallImpactDetector implements BNO055IMU.AccelerationIntegrator {
    private static final double THRESHOLD = 9.8;
    private final BNO055IMU.AccelerationIntegrator realIntegrator;
    private final Telemetry telemetry;
    private boolean impact = false;

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
        Acceleration acceleration = realIntegrator.getAcceleration();
        final double Magnitude = Math.sqrt(Math.pow(acceleration.xAccel, 2) + Math.pow(acceleration.yAccel, 2) + Math.pow(acceleration.zAccel, 2));

        if (Magnitude > THRESHOLD) {
            telemetry.addData("Impact Detected,", "Acceleration Magnitude: %d", Magnitude);
            telemetry.update();
            setImpact(true);

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
            impact = impact;
        }
    }
}