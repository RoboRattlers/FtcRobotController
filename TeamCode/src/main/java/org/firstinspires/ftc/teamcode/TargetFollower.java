package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.MoreMath.clamp;
import static org.firstinspires.ftc.teamcode.util.MoreMath.clampMagnitude;
import static org.firstinspires.ftc.teamcode.util.MoreMath.map;

import com.qualcomm.robotcore.util.ElapsedTime;

public class TargetFollower {

    private final ElapsedTime runtime = new ElapsedTime();

    private double targetPosition = 0;
    private double currentPosition = 0;
    private double currentVelocity = 0;

    private double maxVelocity;
    private double minVelocityMult = 0;
    private double maxAcceleration;
    private double currentAcceleration = 0;
    private double velocityDampeningThreshold = 3;
    private double tolerance = 0.3;

    private double lastTime = runtime.seconds();
    private double lastPositionSetTime = 0;

    public TargetFollower(double vel, double acc, double threshold) {
        maxVelocity = vel;
        maxAcceleration = acc;
        velocityDampeningThreshold = threshold;
    }

    public boolean reachedTarget() {
        return Math.abs(targetPosition - currentPosition) < tolerance;
    }

    public double getCurrentVelocity() {
        return currentVelocity;
    }

    public double getCurrentAcceleration() {
        return currentAcceleration;
    }

    public double getCurrentPosition() {
        return currentPosition;
    }

    public void setTargetPosition(double pos) {
        this.targetPosition = pos;
    }
    public void setMaxAcceleration(double maxAcceleration) {
        this.maxAcceleration = maxAcceleration;
    }
    public void setMaxVelocity(double maxVelocity) {
        this.maxVelocity = maxVelocity;
    }
    public void setMinVelocityMult(double minVelocityMult) {
        this.minVelocityMult = minVelocityMult;
    }
    public void setVelocityDampeningThreshold(double velocityDampeningThreshold) {
        this.velocityDampeningThreshold = velocityDampeningThreshold;
    }

    public void update() {
        double deltaTime = runtime.seconds() - lastTime;
        lastTime = runtime.seconds();

        currentAcceleration = maxAcceleration; //clamp(currentAcceleration + deltaTime * 20 * maxAcceleration, 0, maxAcceleration);
        currentVelocity = clamp(
                currentVelocity + Math.signum(targetPosition - currentPosition) * (currentAcceleration * deltaTime),
                -maxVelocity,
                maxVelocity
        );
        double currentMaxVelocity = maxVelocity * map(
                clamp(Math.abs(targetPosition - currentPosition), 0, velocityDampeningThreshold),
                velocityDampeningThreshold, 0, 1, minVelocityMult, true
                );
        currentVelocity = clampMagnitude(currentVelocity, currentMaxVelocity);
        currentVelocity = clampMagnitude(currentVelocity, (targetPosition - currentPosition)/deltaTime);


        currentPosition += currentVelocity * deltaTime;
    }

    public void update(double overridePosition) {
        currentPosition = overridePosition;
        update();
    }

    public void update(double overridePosition, double overrideVelocity) {
        currentPosition = overridePosition;
        currentVelocity = overrideVelocity;
        update();
    }

    public double getTargetPosition() {
        return targetPosition;
    }
}
