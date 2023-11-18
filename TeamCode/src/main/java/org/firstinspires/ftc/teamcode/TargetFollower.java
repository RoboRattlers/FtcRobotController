package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.MoreMath.clamp;
import static org.firstinspires.ftc.teamcode.util.MoreMath.map;

import com.qualcomm.robotcore.util.ElapsedTime;

public class TargetFollower {

    private final ElapsedTime runtime = new ElapsedTime();

    public double targetPosition = 0;
    private double currentPosition = 0;
    private double currentVelocityWithoutP = 0;
    private double currentVelocity = 0;

    public double maxVelocity; // in/sec
    public double acceleration; // in/sec^2

    public double velocityDampeningThreshold = 3;
    public double tolerance = 0.3;

    private double lastTime = runtime.seconds();

    public TargetFollower(double vel, double acc) {
        maxVelocity = vel;
        acceleration = acc;
    }

    public boolean reachedTarget() {
        return Math.abs(targetPosition - currentPosition) < tolerance;
    }

    public double getCurrentVelocity() {
        return currentVelocity;
    }

    public void update() {
        double deltaTime = runtime.seconds() - lastTime;
        lastTime = runtime.seconds();

        currentVelocityWithoutP += clamp(Math.signum(targetPosition - currentPosition) * (acceleration * deltaTime), -maxVelocity, maxVelocity);
        currentVelocity = currentVelocityWithoutP * map(
                clamp(Math.abs(targetPosition - currentPosition), 0, velocityDampeningThreshold),
                velocityDampeningThreshold, 0, 1, 0.1, false
                );

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

}
