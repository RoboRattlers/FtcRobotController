package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.util.MoreMath.*;

import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public abstract class LinearSlide {


    double fullLength; // in;
    double cappedLength; // in;
    double encoderTicksPerRevolution;
    double encoderTicksInFullLength;
    private final double encoderTicksPerInch = encoderTicksInFullLength/fullLength;
    private final ElapsedTime runtime = new ElapsedTime();

    double maxVelocity = 25; // in/sec
    double acceleration = maxVelocity/0.2; // in/sec^2

    DcMotorEx motor;
    private final double heightTolerance = 0.1;
    private final double velocityDampeningThreshold = 3;

    private double targetHeight = 0;
    private double currentHeight = 0;
    private double currentVelocityWithoutP = 0;
    private double currentVelocity = 0;

    private double lastTime = runtime.seconds();


    public boolean reachedTarget() {
        return Math.abs(targetHeight - currentHeight) < heightTolerance;
    }

    public double getMotorPosForHeight(double height) {
        return height * encoderTicksPerInch;
    }

    public void setTargetHeight(double val) {
        targetHeight = Math.min(Math.max(val, 0), cappedLength);
    }

    public void update() {

        double deltaTime = runtime.seconds() - lastTime;
        lastTime = runtime.seconds();

        currentHeight = motor.getCurrentPosition()/encoderTicksInFullLength;
        currentVelocityWithoutP += clamp(Math.signum(targetHeight - currentHeight) * (acceleration * deltaTime), -maxVelocity, maxVelocity);
        currentVelocity = currentVelocityWithoutP * map(
                clamp(Math.abs(targetHeight - currentHeight), 0, velocityDampeningThreshold),
                velocityDampeningThreshold, 0, 1, 0.1, false
                );

        double velocityInEncoderTicks = encoderTicksPerInch * currentVelocity;
        motor.setVelocity(velocityInEncoderTicks/encoderTicksPerRevolution * Math.PI * 2, AngleUnit.RADIANS);

    }

}
