package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.MoreMath.clamp;
import static org.firstinspires.ftc.teamcode.util.MoreMath.map;

import androidx.core.math.MathUtils;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

class ClawPose {

    public static final double[] DRIVE = new double[]{0.5, 0.22};
    public static final double[] DRIVE_LOW = new double[]{0.5, 0.3};
    public static final double[] GRAB = new double[]{0.4, 0.6};
    public static final double[] LOAD = new double[]{0.7, 0.44};
    public static final double[] FORWARD = new double[]{0.5, 0.5};
}

class PusherPosition {
    public static final double FULLY_RETRACTED = 0.33;
    public static final double ZERO_PIXEL = 0.25;
    public static final double ONE_PIXEL = 0.168;
    public static final double SAFE_ONE_PIXEL = 0.2;
    public static final double TWO_PIXEL = 0.13;
}

public class CenterStageHardware {
    public Servo clawShoulder;
    public Servo clawElbow;
    public Servo pokerPusher;
    public DcMotorEx armRotator;
    public Servo pokerRotator;
    public DcMotorEx armExtender;
    public Servo clawOpener;
    public Servo clawShelf;
    public Servo droneLauncher;
    public Rev2mDistanceSensor backboardDistanceSensor;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private TargetFollower armExtenderFollower = new TargetFollower(2000, 5000, 10); // 0.9 = 90 degrees
    private TargetFollower armRotatorFollower = new TargetFollower(4000, 20000, 100); // 4 = full extension
    private TargetFollower clawShoulderFollower = new TargetFollower(10, 10, 0.025); // normal servo parameters
    private TargetFollower clawElbowFollower = new TargetFollower(5, 10, 0.025);
    public double pusherPos = PusherPosition.FULLY_RETRACTED;
    public double[] clawPose = ClawPose.DRIVE;
    public boolean clawOpen = false;
    public boolean smoothArmExtension = true;

    public CenterStageHardware(HardwareMap hardware, Telemetry tele) {
        hardwareMap = hardware;
        telemetry = tele;
        clawShoulder = hardwareMap.get(Servo.class, "ClawShoulder"); // expansion hub servo 0
        clawElbow = hardwareMap.get(Servo.class, "ClawElbow"); // expansion hub servo 1
        clawOpener = hardwareMap.get(Servo.class, "ClawOpener"); // expansion hub servo 3
        pokerPusher = hardwareMap.get(Servo.class, "PokerPusher"); // control hub servo 0
        pokerRotator = hardwareMap.get(Servo.class, "PokerRotator"); // control hub servo 1
        armRotator = hardwareMap.get(DcMotorEx.class, "ArmRotator"); // expansion hub motor 0
        armExtender = hardwareMap.get(DcMotorEx.class,  "ArmExtender"); // expansion hub motor 1
        clawShelf = hardwareMap.get(Servo.class, "ClawShelf");
        droneLauncher = hardwareMap.get(Servo.class, "DroneLauncher");
        backboardDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "BackboardDistanceSensor");

        pokerPusher.setPosition(PusherPosition.FULLY_RETRACTED);
        pokerRotator.setPosition(0.5);

        armExtender.setTargetPosition(0);
        armExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtender.setPower(1);
        armExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armRotator.setTargetPosition(0);
        armRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRotator.setPower(1);
        armRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setTargetArmRotation(double pos) {
        armRotatorFollower.setTargetPosition(clamp(pos, 0, 1300));
    }
    public void setTargetArmExtension(double pos) {
        armExtenderFollower.setTargetPosition(clamp(pos, -7500, 0));
    }

    public double getTargetArmExtension() {
        return armExtenderFollower.getTargetPosition();
    }

    public void setTargetPokerRotation(double pos) {
        pokerRotator.setPosition(pos);
    }

    private final Vector2d pokerEndpointOffset = new Vector2d(-53, 200);
    private final Vector2d rotatorShaftXOffset = new Vector2d(-32, 0);
    private final Vector2d rotatorShaftYOffset = new Vector2d(0, 240);

    private final double armCountPerRotation = ((1.0+(46.0/11.0)) * 28.0) * 28.0;
    private final double pokerRotatorCountPerRotation = 1.0/(300.0/360.0);
    private final double armCountPerMm = 1425.1/120; //120mm of distance on the pulley
    private final double ARM_RETRACTED_LENGTH = 240;

    private final double ELBOW_OFFSET = 0.06;
    private final double initialRotatorShaftAngle = Math.atan2(rotatorShaftYOffset.getY(), rotatorShaftXOffset.getX());
    public void setIKPokerEndpoint(Pose2d pokerEndpoint) {
        Vector2d rotatorShaftEndpoint = pokerEndpoint.vec().minus(pokerEndpointOffset.rotated(pokerEndpoint.getHeading()));
        double rotatorShaftAngle = Math.atan2(rotatorShaftEndpoint.getY(), rotatorShaftEndpoint.getX());
        Vector2d armEndpoint = rotatorShaftEndpoint.minus(rotatorShaftXOffset.rotated(rotatorShaftAngle - initialRotatorShaftAngle));
        double armRotation = Math.atan2(armEndpoint.getY(), armEndpoint.getX());
        double armLength = armEndpoint.distTo(new Vector2d());

        armRotatorFollower.setTargetPosition(MathUtils.clamp((int) (armRotation / (Math.PI * 2) * armCountPerRotation), 50, 1600));
        armExtenderFollower.setTargetPosition((int) MathUtils.clamp(-(armLength - ARM_RETRACTED_LENGTH) * armCountPerMm, -6000, 0));
        double realPokerRotation = (pokerEndpoint.getHeading()/ (Math.PI * 2) ) - (armRotation / (Math.PI * 2) - 0.25);
        pokerRotator.setPosition((-realPokerRotation) * pokerRotatorCountPerRotation + 0.5);
    }

    public void initPose() {
        clawOpen = false;
        clawPose = ClawPose.DRIVE;

        setTargetArmRotation(0);
        setTargetArmExtension(0);
        pokerRotator.setPosition(0.175);
        pusherPos = PusherPosition.FULLY_RETRACTED;
        droneLauncher.setPosition(0.64);

        update(true);
    }

    public static double[] getClawPoseForHeight(double height) {
        height -= 30.7; // account for distance from ground when straight forward
        height /= 72; // account for length of thingy
        if (Math.abs(height) > 1) {
            return new double[]{0.5, 0.5};
        }
        double angle = Math.asin(height);
        return new double[]{
                map(angle, Math.toRadians(150), Math.toRadians(-150), 1, 0, true),
                0.52
        };
    }

    public void update(boolean instantaneous) {
        armRotatorFollower.update();
        armExtenderFollower.update();
        telemetry.addData("Arm Rotator Target Pos", armRotatorFollower.getTargetPosition());
        telemetry.addData("Arm Rotator Current Pos", armRotatorFollower.getCurrentPosition());
        telemetry.addData("Arm Extender Target Pos", armExtenderFollower.getTargetPosition());
        telemetry.addData("Arm Extender Current Pos", armExtenderFollower.getCurrentPosition());
        armExtender.setTargetPosition((int) (clamp(
                (!smoothArmExtension || instantaneous) ? armExtenderFollower.getTargetPosition() : armExtenderFollower.getCurrentPosition()
                , -7500, 0)));
        armRotator.setTargetPosition((int) clamp((instantaneous ? armRotatorFollower.getTargetPosition() : armRotatorFollower.getCurrentPosition()), 0, 1500));
        pokerPusher.setPosition(pusherPos);
        ;
        clawShoulderFollower.setTargetPosition(clawPose[0]);
        clawElbowFollower.setTargetPosition(clawPose[1] + ELBOW_OFFSET);
        clawShoulderFollower.update();
        clawElbowFollower.update();
        clawShoulder.setPosition(instantaneous ? clawShoulderFollower.getTargetPosition() : clawShoulderFollower.getCurrentPosition());
        clawElbow.setPosition(instantaneous ? clawElbowFollower.getTargetPosition() : clawElbowFollower.getCurrentPosition());
        clawOpener.setPosition(clawOpen ? 0.3 : 0.55);
    }

}
