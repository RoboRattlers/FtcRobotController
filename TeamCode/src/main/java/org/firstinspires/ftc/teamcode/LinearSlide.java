package org.firstinspires.ftc.teamcode;

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
    double encoderTicksPerInch;
    private final ElapsedTime runtime = new ElapsedTime();
    private final TargetFollower follower = new TargetFollower(25 * encoderTicksPerInch, 200 * encoderTicksPerInch, 50);

    double maxVelocity = 25; // in/sec
    double acceleration = maxVelocity/0.2; // in/sec^2

    DcMotorEx motor;
    private final double heightTolerance = 0.1;
    private final double velocityDampeningThreshold = 0.1;

    private double targetHeight = 0;
    private double currentHeight = 0;
    private double currentVelocityWithoutP = 0;
    private double currentVelocity = 0;

    private double lastTime = runtime.seconds();


    public boolean reachedTarget() {
        return follower.reachedTarget();
    }

    public double getMotorPosForHeight(double height) {
        return height * encoderTicksPerInch;
    }

    public void setTargetHeight(double val) {
        targetHeight = Math.min(Math.max(val, 0), cappedLength);
    }

    public double getTargetHeight() {
        return targetHeight;
    }

    public void update() {

        encoderTicksPerInch = Math.abs(encoderTicksInFullLength/fullLength);

        double deltaTime = runtime.seconds() - lastTime;
        lastTime = runtime.seconds();

        targetHeight = clamp(targetHeight, 0, cappedLength);
        follower.setTargetPosition(targetHeight/fullLength * encoderTicksInFullLength);
        follower.update(motor.getCurrentPosition(), motor.getVelocity());

        double velocityInEncoderTicks = encoderTicksPerInch * follower.getCurrentVelocity();
        //motor.setVelocity(velocityInEncoderTicks/encoderTicksPerRevolution * Math.PI * 2, AngleUnit.RADIANS);

        motor.setTargetPosition((int) (encoderTicksPerInch * targetHeight * -1));

    }

}
